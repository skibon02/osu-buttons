use core::sync::atomic::AtomicBool;
use cortex_m::peripheral::NVIC;
use defmt::Debug2Format;
use crate::dev::interrupt;
use stm32f1::stm32f103::{GPIOC, RCC, USB};
use crate::util::delay_us;

static Z_PRESSED: AtomicBool = AtomicBool::new(false);
static X_PRESSED: AtomicBool = AtomicBool::new(false);

static NEED_UPDATE: AtomicBool = AtomicBool::new(false);

pub struct UsbController {
    usb: USB,
    gpioc: GPIOC,
}

impl UsbController {
    pub fn init(rcc: &RCC, usb: USB, gpioc: GPIOC) -> Self {
        rcc.apb2enr().modify(|r, w| w
            .iopaen().set_bit());
        rcc.apb1enr().modify(|r, w| w
            .usben().set_bit());

        // Clear PDWN to enable USB transceiver
        usb.cntr().modify(|r, w| w
            .pdwn().clear_bit());

        // Wait for tSTARTUP
        delay_us(10);

        // Clear FRES to remove USB reset condition
        usb.cntr().modify(|_, w| w
            .fres().no_reset());

        // Clear ISTR to remove any spurious pending interrupts
        usb.istr().write(|w| unsafe { w.bits(0) });

        // Initialize packet buffer memory and buffer descriptor table
        unsafe {
            // Clear entire packet memory (512 bytes = 256 u16 words)
            let packet_mem_base = 0x40006000 as *mut u16;
            for i in 0..256 {
                packet_mem_base.add(i).write_volatile(0);
            }
        }

        // Set Buffer Table to start of packet memory
        unsafe { usb.btable().write(|w| w.bits(0)); }

        bm_write_u16(0, 0x0100);     // ADDR0_TX = 0x0050
        bm_write_u16(2, 0);          // COUNT0_TX = 0
        bm_write_u16(4, 0x0040);     // ADDR0
        bm_write_u16(6, 0x8800);     // COUNT0_RX = blocks | 3 =>  128 bytes

        // EP1: Only TX for HID IN endpoint
        bm_write_u16(8, 0x0180);     // ADDR1_TX = 0x00C0
        bm_write_u16(10, 0);         // COUNT1_TX = 0
        bm_write_u16(12, 0);
        bm_write_u16(14, 0);

        usb.cntr().modify(|_, w| w
            .errm().set_bit()
            .resetm().set_bit()
            .ctrm().set_bit());

        unsafe { NVIC::unmask(interrupt::USB_HP_CAN_TX); }
        unsafe { NVIC::unmask(interrupt::USB_LP_CAN_RX0); }

        info!("USB initialized");
        Self {
            usb,
            gpioc
        }
    }

    pub fn set_z_pressed(&self, pressed: bool) {
        Z_PRESSED.store(pressed, core::sync::atomic::Ordering::Relaxed);

        let z = Z_PRESSED.load(core::sync::atomic::Ordering::Relaxed);
        let x = X_PRESSED.load(core::sync::atomic::Ordering::Relaxed);
        self.gpioc.odr().modify(|_,  w| w.odr13().bit(!x && !z));

        update_report();
    }
    pub fn set_x_pressed(&self, pressed: bool) {
        X_PRESSED.store(pressed, core::sync::atomic::Ordering::Relaxed);

        let z = Z_PRESSED.load(core::sync::atomic::Ordering::Relaxed);
        let x = X_PRESSED.load(core::sync::atomic::Ordering::Relaxed);
        self.gpioc.odr().modify(|_,  w| w.odr13().bit(!x && !z));

        update_report();
    }

}

pub fn update_report() {
    let z = Z_PRESSED.load(core::sync::atomic::Ordering::Relaxed);
    let x = X_PRESSED.load(core::sync::atomic::Ordering::Relaxed);
    let report: [u8; 8] = [
        0,          // Modifier byte
        0,          // Reserved
        if z { 0x1D } else { 0 }, // 'Z'
        if x { 0x1B } else { 0 }, // 'X'
        0, 0, 0, 0  // Remaining keycodes
    ];
    let tx_addr = bm_read_u16(8); // ADDR1_TX
    bm_write_bytes(tx_addr as usize, &report);
    bm_write_u16(10, report.len() as u16); // COUNT1_TX
    info!("\t* HID report updated: {:x}", report);

    set_ep_stat_tx(1, 0b11); // VALID
}
fn bm_read_u16(byte_offset: usize) -> u16 {
    let packet_mem_base = 0x40006000 as *const u16;
    unsafe { packet_mem_base.add(byte_offset).read_volatile() }
}

fn bm_write_u16(byte_offset: usize, value: u16) {
    let packet_mem_base = 0x40006000 as *mut u16;
    unsafe { packet_mem_base.add(byte_offset).write_volatile(value); }
}

fn bm_read_u32(byte_offset: usize) -> u32 {
    let u16_lo = bm_read_u16(byte_offset) as u32;
    let u16_hi = bm_read_u16(byte_offset + 2) as u32;
    (u16_hi << 16) | u16_lo
}

fn bm_write_u32(byte_offset: usize, value: u32) {
    let u16_lo = (value & 0xFFFF) as u16;
    let u16_hi = ((value >> 16) & 0xFFFF) as u16;
    bm_write_u16(byte_offset, u16_lo);
    bm_write_u16(byte_offset + 2, u16_hi);
}

fn bm_write_bytes(byte_offset: usize, data: &[u8]) {
    assert!(data.len() + byte_offset <= 512, "Packet memory write out of bounds!");
    for (i, b) in data.chunks(2).enumerate() {
        let word = if b.len() == 2 {
            (b[1] as u16) << 8 | (b[0] as u16)
        } else {
            b[0] as u16
        };
        bm_write_u16(byte_offset + i * 2, word);
    }
}

fn bm_read_bytes(byte_offset: usize, buf: &mut [u8]) {
    assert!(buf.len() + byte_offset <= 512, "Packet memory read out of bounds!");
    for (i, b) in buf.chunks_mut(2).enumerate() {
        let word = bm_read_u16(byte_offset + i * 2);
        if b.len() == 2 {
            b[0] = (word & 0xFF) as u8;
            b[1] = (word >> 8) as u8;
        } else {
            b[0] = (word & 0xFF) as u8;
        }
    }
}

#[cortex_m_rt::interrupt]
fn USB_HP_CAN_TX() {
    info!("TX interrupt");

}

fn set_ep_stat_tx(ep_num: usize, stat: u8) {
    let usb = &unsafe {USB::steal()};
    let mut v = usb.epr(ep_num).read().bits();
    //zero dtog
    v &= !(0b1 << 6);
    v &= !(0b1 << 14);

    // zero stat_rx
    v &= !(0b11 << 12);

    // set stat
    v ^= (stat as u32) << 4;

    usb.epr(ep_num).write(|w| unsafe { w.bits(v) });
}

fn set_ep_stat_rx(ep_num: usize, stat: u8) {
    let usb = &unsafe {USB::steal()};
    let mut v = usb.epr(ep_num).read().bits();
    //zero dtog
    v &= !(0b1 << 6);
    v &= !(0b1 << 14);

    // zero stat_tx
    v &= !(0b11 << 4);

    // set stat
    v ^= (stat as u32) << 12;

    usb.epr(ep_num).write(|w| unsafe { w.bits(v) });
}

fn ep_clear_ctr_rx(ep_num: usize) {
    let usb = &unsafe {USB::steal()};
    let mut v = usb.epr(ep_num).read().bits();
    v &= !(0b0111_0000_0111_0000); // zero toggle bits
    v &= !(0b1 << 15);  // zero CTR_RX
    usb.epr(ep_num).write(|w| unsafe { w.bits(v) });
}
fn ep_clear_ctr_tx(ep_num: usize) {
    let usb = &unsafe {USB::steal()};
    let mut v = usb.epr(ep_num).read().bits();
    v &= !(0b0111_0000_0111_0000); // zero toggle bits
    v &= !(0b1 << 7); // zero CTR_TX
    usb.epr(ep_num).write(|w| unsafe { w.bits(v) });
}

#[cortex_m_rt::interrupt]
fn USB_LP_CAN_RX0() {
    static mut PENDING_ADDRESS: Option<u8> = None;

    info!("\n\n\t\t[RX0 interrupt]");

    let usb = &unsafe {USB::steal()};
    let istr = usb.istr().read();
    info!("istr: {:#?}", Debug2Format(&istr));
    let ep0r_val = usb.ep0r().read();

    if istr.err().bit_is_set() {
        usb.istr().modify(|_, w| w.err().clear_bit());
        warn!("USB error interrupt");
    }
    if istr.ctr().bit_is_set() {
        let ep_num = istr.ep_id().bits();
        let dir = istr.dir().bit();
        if dir {
            info!(" > FROM HOST, EP{}", ep_num);
        }
        else {
            info!(" > TO HOST, EP{}", ep_num);
        }

        if ep_num == 0 {
            info!("EP0R register: {:08X}", ep0r_val.bits());

            info!("EP0 RX: {}\n\tstatus: {}\n\tdata toggle: {}", ep0r_val.ctr_rx().bit_is_set(), Debug2Format(&ep0r_val.stat_rx().variant()), ep0r_val.dtog_rx().bit());
            info!("EP0 TX: {}\n\tstatus: {}\n\tdata toggle: {}", ep0r_val.ctr_tx().bit_is_set(), Debug2Format(&ep0r_val.stat_tx().variant()), ep0r_val.dtog_tx().bit());

            if ep0r_val.ctr_rx().bit_is_set() {
                ep_clear_ctr_rx(0);

                // Read transaction packet from RX buffer
                let rx_offset = bm_read_u16(4); // ADDR0
                let rx_orig_count = bm_read_u16(6);  // COUNT0_RX
                assert!(rx_offset < 256, "RX buffer address out of range!");

                let rx_count = rx_orig_count & 0x3FF;

                let mut transaction_buffer = [0u8; 128];
                bm_read_bytes(rx_offset as usize, &mut transaction_buffer[0..rx_count as usize]);

                if ep0r_val.setup().bit_is_set() {
                    let setup_data = &transaction_buffer[0..8];

                    let bm_request_type = setup_data[0];
                    let b_request = setup_data[1];
                    let w_value = (setup_data[3] as u16) << 8 |
                        (setup_data[2] as u16);
                    let w_index = (setup_data[5] as u16) << 8 |
                        (setup_data[4] as u16);
                    let w_length = (setup_data[7] as u16) << 8 |
                        (setup_data[6] as u16);
                    info!("\t* SETUP packet: bmRequestType={:02X}, bRequest={:02X}, wValue={:04X}, wIndex={:04X}, wLength={}",
                          bm_request_type, b_request, w_value, w_index, w_length);

                    match (bm_request_type, b_request) {
                        // GET_DESCRIPTOR
                        (0x80, 0x06) => {
                            let descriptor_type = ((w_value >> 8) & 0xFF) as u8;
                            let _descriptor_index = (w_value & 0xFF) as u8;

                            let descriptor_bytes = match descriptor_type {
                                1 => DEVICE_DESCRIPTOR,
                                2 => CONFIGURATION_DESCRIPTOR,
                                6 => DEVICE_QUALIFIER_DESCRIPTOR,
                                0x22 => HID_REPORT_DESCRIPTOR,
                                _ => {
                                    todo!("Unknown descriptor type {:02X}", descriptor_type);
                                }
                            };

                            let send_len = core::cmp::min(descriptor_bytes.len(), w_length as usize);
                            info!("\t** GET_DESCRIPTOR type {:02X}, sending {} bytes", descriptor_type, send_len);

                            // Copy data to TX buffer
                            let tx_addr = bm_read_u16(0); // ADDR0_TX
                            bm_write_bytes(tx_addr as usize, &descriptor_bytes[0..send_len]);
                            // Set COUNT0_TX
                            bm_write_u16(2, send_len as u16);

                            // Enable TX
                            set_ep_stat_tx(0, 0b11); // VALID
                            info!("Marked tx valid");
                        },
                        // SET_ADDRESS
                        (0x00, 0x05) => {
                            let address = (w_value & 0x7F) as u8;
                            info!("\t* SET_ADDRESS: {}", address);

                            PENDING_ADDRESS.replace(address);

                            // Send zero-length status packet first
                            // Set COUNT0_TX to 0
                            bm_write_u16(2, 0);
                            set_ep_stat_tx(0, 0b11); // VALID
                        },
                        // SET_CONFIGURATION
                        (0x00, 0x09) => {
                            let configuration_value = (w_value & 0xFF) as u8;
                            info!("\t* SET_CONFIGURATION: {}", configuration_value);

                            // Send zero-length status packet
                            bm_write_u16(2, 0);
                            set_ep_stat_tx(0, 0b11); // VALID
                        },
                        // GET_STATUS
                        (0x80, 0x00) => {
                            todo!("Handle GET_STATUS request");
                        },
                        (0x21, 0x09) => {
                            // HID SET_REPORT
                            info!("\t* HID SET_REPORT");
                            // Send zero-length status packet
                            bm_write_u16(2, 0);
                            set_ep_stat_tx(0, 0b11); // VALID
                        }
                        (0x21, 0x0A) => {
                            // HID SET_IDLE
                            info!("\t* HID SET_IDLE");
                            // Send zero-length status packet
                            bm_write_u16(2, 0);
                            set_ep_stat_tx(0, 0b11); // VALID
                        },
                        (0x81, 0x06) => {
                            // HID GET_DESCRIPTOR
                            let descriptor_type = ((w_value >> 8) & 0xFF) as u8;
                            let _descriptor_index = (w_value & 0xFF) as u8;

                            let descriptor_bytes = match descriptor_type {
                                0x22 => HID_REPORT_DESCRIPTOR,
                                _ => {
                                    todo!("Unknown HID descriptor type {:02X}", descriptor_type);
                                }
                           };

                            let send_len = core::cmp::min(descriptor_bytes.len(), w_length as usize);
                            info!("\t** HID GET_DESCRIPTOR type {:02X}, sending {} bytes", descriptor_type, send_len);

                            // Copy data to TX buffer
                            let tx_addr = bm_read_u16(0); // ADDR0_TX
                            bm_write_bytes(tx_addr as usize, &descriptor_bytes[0..send_len]);
                            // Set COUNT0_TX
                            bm_write_u16(2, send_len as u16);

                            // Enable TX
                            set_ep_stat_tx(0, 0b11); // VALID
                            info!("Marked tx valid");
                        }
                        (0x02, 0x01) => {
                            // SET_FEATURE (remote wakeup)
                            info!("\t* SET_FEATURE");

                            // Send zero-length status packet
                            bm_write_u16(2, 0);
                            set_ep_stat_tx(0, 0b11); // VALID
                        }
                        _ => {
                            // STALL unknown requests
                            error!("\t* Unknown SETUP: bmRequestType={:02X}, bRequest={:02X}", bm_request_type, b_request);
                            set_ep_stat_tx(0, 0b01); // STALL
                        }
                    }
                }
                else {
                    let transaction_data = &transaction_buffer[0..rx_count as usize];
                    info!("\t* IN packet: {:x}", transaction_data);

                    set_ep_stat_tx(0, 0b11); // VALID
                }
            }

            if ep0r_val.ctr_tx().bit_is_set() {
                ep_clear_ctr_tx(0);

                // handle set address
                if let Some(addr) = PENDING_ADDRESS.take() {
                    info!("\t* Setting device address to {}", addr);

                    // Set address after status stage
                    usb.daddr().write(|w| w.ef().set_bit().add().set(addr));

                }

                info!("\t* TX complete, rx mark as valid");

                set_ep_stat_rx(0, 0b11); // VALID
            }
        }
        else if ep_num == 1 {
            let ep1r_val = usb.epr(1).read();
            if ep1r_val.ctr_tx().bit_is_set() {
                ep_clear_ctr_tx(1);

            }

            if ep1r_val.ctr_rx().bit_is_set() {
                ep_clear_ctr_rx(1);

                panic!("Unexpected RX on EP1")
            }
        }
        else {
            panic!("Unknown endpoint number {}", ep_num);
        }
    }

    if istr.reset().bit_is_set() {
        usb.istr().modify(|_, w| w.reset().clear_bit());
        info!("USB reset");

        // Configure EP0 register
        usb.epr(0).write(|w|
            w.ea().set(0)
                .ep_type().control()
                .stat_rx().valid()
                .stat_tx().nak()
        );

        // Configure EP1 register (HID IN)
        usb.epr(1).write(|w|
            w.ea().set(1)
                .ep_type().interrupt()
                .stat_rx().disabled()
                .stat_tx().valid()
        );

        // Enable device with address 0
        unsafe { usb.daddr().write(|w| w.ef().set_bit().add().bits(0)); }

        info!("USB device configured");
    }
    if istr.susp().bit_is_set() {
        usb.istr().modify(|_, w| w.susp().clear_bit());
        info!("USB suspend");
    }
}

const DEVICE_DESCRIPTOR: &[u8] = &[
    18,    // bLength
    1,     // bDescriptorType (Device)
    0x00, 0x02, // bcdUSB (USB 2.0)
    0,     // bDeviceClass
    0,     // bDeviceSubClass
    0,     // bDeviceProtocol
    64,    // bMaxPacketSize0
    0x83, 0x04, // idVendor (0x0483 - STMicroelectronics)
    0x40, 0x57, // idProduct (0x5740)
    0x00, 0x01, // bcdDevice (1.0)
    0,     // iManufacturer
    0,     // iProduct
    0,     // iSerialNumber
    1      // bNumConfigurations
];

const CONFIGURATION_DESCRIPTOR: &[u8] = &[
    // Configuration Descriptor
    9,     // bLength
    2,     // bDescriptorType (Configuration)
    34, 0, // wTotalLength (9+9+9+7 = 34 bytes)
    1,     // bNumInterfaces
    1,     // bConfigurationValue
    0,     // iConfiguration
    0x80,  // bmAttributes (bus powered)
    250,   // bMaxPower (500mA)

    // Interface Descriptor
    9,     // bLength
    4,     // bDescriptorType (Interface)
    0,     // bInterfaceNumber
    0,     // bAlternateSetting
    1,     // bNumEndpoints
    3,     // bInterfaceClass (HID)
    1,     // bInterfaceSubClass (Boot)
    1,     // bInterfaceProtocol (Keyboard)
    0,     // iInterface

    // HID Descriptor
    9,     // bLength
    0x21,  // bDescriptorType (HID)
    0x11, 0x01, // bcdHID (HID 1.11)
    0,     // bCountryCode
    1,     // bNumDescriptors
    0x22,  // bDescriptorType (Report)
    63, 0, // wDescriptorLength (63 bytes)

    // Endpoint Descriptor
    7,     // bLength
    5,     // bDescriptorType (Endpoint)
    0x81,  // bEndpointAddress (EP1 IN)
    3,     // bmAttributes (Interrupt)
    8, 0,  // wMaxPacketSize (8 bytes)
    1      // bInterval (1ms for 1000Hz polling)
];

const DEVICE_QUALIFIER_DESCRIPTOR: &[u8] = &[
    10,    // bLength
    6,     // bDescriptorType (Device Qualifier)
    0x00, 0x02, // bcdUSB (USB 2.0)
    0,     // bDeviceClass
    0,     // bDeviceSubClass
    0,     // bDeviceProtocol
    64,    // bMaxPacketSize0
    1,     // bNumConfigurations
    0      // bReserved
];

const HID_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xa1, 0x01,        // Collection (Application)
    0x05, 0x07,        //   Usage Page (Keyboard)
    0x19, 0xe0,        //   Usage Minimum (224)
    0x29, 0xe7,        //   Usage Maximum (231)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Cnst,Ary,Abs)
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x05,        //   Usage Maximum (5)
    0x91, 0x02,        //   Output (Data,Var,Abs)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x01,        //   Output (Cnst,Ary,Abs)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Keyboard)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x81, 0x00,        //   Input (Data,Ary,Abs)
    0xc0               // End Collection
];

