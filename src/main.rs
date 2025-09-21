#![no_main]
#![no_std]

pub mod usb;
pub mod rcc;
mod util;

use core::sync::atomic::Ordering;
use cortex_m::asm::delay;
use embedded_alloc::LlffHeap;
use stm32f1::stm32f103 as dev;
use crate::rcc::switch_to_pll;
use crate::usb::{update_report, UsbController};
use crate::util::{delay_ms, delay_us};

#[macro_use(info, debug, warn, error)]
extern crate defmt;

extern crate panic_probe;
extern crate defmt_rtt;

const HEAP: LlffHeap = LlffHeap::empty();

#[cortex_m_rt::entry]
fn main() -> !{
    unsafe { HEAP.init(0x2000_2400, 1024); }

    let p = dev::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // enable cycle counter
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();


    p.RCC.apb2enr().modify(|_, w| w.iopaen().set_bit());

    // setup pa6, pa7 as input pull-up pins
    p.GPIOA.crl().modify(|_, w| w.mode6().input().mode7().input()); // input mode
    p.GPIOA.crl().modify(|_, w| unsafe {w.cnf6().bits(0b10).cnf7().bits(0b10)}); // input with pull-up/pull-down
    p.GPIOA.bsrr().write(|w| w.bs6().set_bit().bs7().set_bit()); // pull-up



    p.RCC.apb2enr().modify(|_, w| w.iopcen().set_bit());
    //setup c13 pin as output
    p.GPIOC.crh().modify(|_, w| w.mode13().output50());
    p.GPIOC.odr().modify(|_,  w| w.odr13().bit(true));

    switch_to_pll(&p.RCC, &p.FLASH);

    let mut usb = UsbController::init(&p.RCC, p.USB, p.GPIOC);

    let mut total_tm = 0u64;
    let mut prev_tm = cp.DWT.cyccnt.read();
    let mut now = || -> u64 {
        let new_tm = cp.DWT.cyccnt.read();
        let elapsed = new_tm.wrapping_sub(prev_tm);
        prev_tm = new_tm;

        total_tm += elapsed as u64;
        total_tm
    };

    let mut prev_ref = now();
    let mut wait_ticks = |ticks: u64| {
        loop {
            let cur = now();
            if cur - prev_ref >= ticks {
                prev_ref += ticks;
                break;
            }
        }
    };
    //
    // loop {
    //     usb.set_x_pressed(true);
    //     wait_ticks(200_000);
    //     usb.set_x_pressed(false);
    //     wait_ticks(4_100_000);
    //
    //     usb.set_z_pressed(true);
    //     wait_ticks(200_000);
    //     usb.set_z_pressed(false);
    //     wait_ticks(4_100_000);
    // }

    let mut prev_pa6 = false;
    let mut prev_pa7 = false;



    let mut total_tm = 0u64;
    let mut prev_tm = cp.DWT.cyccnt.read();
    let mut now = || -> u64 {
        let new_tm = cp.DWT.cyccnt.read();
        let elapsed = new_tm.wrapping_sub(prev_tm);
        prev_tm = new_tm;

        total_tm += elapsed as u64;
        total_tm
    };
    let release_ignore_win = util::SYSTEM_CLOCK_KHZ.load(Ordering::Relaxed) as u64 * 20; // 30ms ignore window

    let mut pa6_prev_change = now();
    let mut pa7_prev_change = now();
    loop {
        let pa6 = p.GPIOA.idr().read().idr6().bit_is_clear();
        let pa7 = p.GPIOA.idr().read().idr7().bit_is_clear();

        if pa6 != prev_pa6 {
            let elapsed = now() - pa6_prev_change;
            if elapsed > release_ignore_win { // 30ms ignore window
                pa6_prev_change = now();
                prev_pa6 = pa6;

                usb.set_z_pressed(pa6);
                update_report();
            }
        }

        if pa7 != prev_pa7 {
            let elapsed = now() - pa7_prev_change;
            if elapsed > release_ignore_win { // 30ms ignore window
                pa7_prev_change = now();
                prev_pa7 = pa7;

                usb.set_x_pressed(pa7);
                update_report();
            }
        }


        wait_ticks(10_000);
    }
}
