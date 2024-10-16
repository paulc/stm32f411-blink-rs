#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use stm32f4xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_log::init();
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).freeze();

    // delay using general-pupose 32-bit timer TIM5
    let mut delay = dp.TIM5.delay_us(&ccdr);

    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();

    let mut count = 0_u32;

    loop {
        log::info!("Count :: {}", count);
        led.toggle();
        delay.delay_ms(200);
        if count == 10 {
            log::error!("Fatal - Panicing");
            panic!("PANIC...");
        }
        count += 1;
    }
}
