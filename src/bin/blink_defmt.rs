#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
use stm32f4xx_hal as hal;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let ccdr = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).freeze();

    // Create a delay abstraction based on general-pupose 32-bit timer TIM5
    let mut delay = dp.TIM5.delay_us(&ccdr);

    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();

    let mut count = 0_u32;
    defmt::info!("Starting...");

    loop {
        defmt::info!("Count: {}", count);
        led.toggle();
        delay.delay_ms(500);
        if count > 10 {
            panic!("PANIC...");
        }
        count += 1;
    }
}
