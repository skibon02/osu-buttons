use stm32f1::stm32f103::{FLASH, RCC};
use crate::util::set_system_clock_khz;

/// Setup clock to 72MHz + USB clock to 48MHz
pub fn switch_to_pll(rcc: &RCC, flash: &FLASH) {
    // Enable HSE
    rcc.cr().modify(|r, w| w.hseon().set_bit());
    while rcc.cr().read().hserdy().bit_is_clear() {}

    // Prepare flash for 72MHz
    flash.acr().modify(|r, w| w.prftbe().set_bit());
    flash.acr().modify(|r, w| w.latency().ws2());

    // configure PLL and prescalers
    rcc.cfgr().modify(|r, w|
        w.hpre().div1()
             .ppre1().div2()
             .ppre2().div1()
             .pllsrc().hse_div_prediv()
             .pllxtpre().div1()
             .pllmul().mul9()
            .usbpre().div1_5()
    );

    // Enable PLL
    rcc.cr().modify(|r, w| w.pllon().set_bit());
    while rcc.cr().read().pllrdy().bit_is_clear() {}

    // Switch system clock to PLL
    rcc.cfgr().modify(|r, w| w.sw().pll());
    while !rcc.cfgr().read().sws().is_pll() {}

    set_system_clock_khz(72_000);
}