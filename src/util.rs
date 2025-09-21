use core::sync::atomic::AtomicU32;

pub static SYSTEM_CLOCK_KHZ: AtomicU32 = AtomicU32::new(4_000);
pub fn set_system_clock_khz(khz: u32) {
    SYSTEM_CLOCK_KHZ.store(khz, core::sync::atomic::Ordering::Relaxed);
}

pub fn delay_us(us: u32) {
    let mhz = SYSTEM_CLOCK_KHZ.load(core::sync::atomic::Ordering::Relaxed) / 1_000;
    let cycles = us.saturating_mul(mhz);
    cortex_m::asm::delay(cycles);
}

pub fn delay_ms(ms: u32) {
    let mhz = SYSTEM_CLOCK_KHZ.load(core::sync::atomic::Ordering::Relaxed);
    let cycles = ms.saturating_mul(mhz);
    cortex_m::asm::delay(cycles);
}

