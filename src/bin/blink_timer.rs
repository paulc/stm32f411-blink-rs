#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use stm32f4xx_hal::{
    gpio::{Edge, Input, Output, PA0, PC13},
    interrupt, pac,
    prelude::*,
    timer::{CounterUs, Event},
};

type Key = PA0<Input>;
type Led = PC13<Output>;
type Tim = CounterUs<pac::TIM2>;

static G_KEY: Mutex<RefCell<Option<Key>>> = Mutex::new(RefCell::new(None));
static G_LED: Mutex<RefCell<Option<Led>>> = Mutex::new(RefCell::new(None));
static G_TIMER: Mutex<RefCell<Option<Tim>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    rtt_log::init();

    // System setup
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).freeze();

    // Needed for interrupt handler
    let mut syscfg = dp.SYSCFG.constrain();
    let mut exti = dp.EXTI;

    // delay using system timer
    let mut _delay = cp.SYST.delay(&clocks);

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    // LED
    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_high();

    // KEY button
    let mut key = gpioa.pa0.into_pull_up_input();

    // Enable interrupt
    key.make_interrupt_source(&mut syscfg);
    key.trigger_on_edge(&mut exti, Edge::Falling);
    key.enable_interrupt(&mut exti);

    // TIMER
    // For <CounterHz> we can use Timer::new
    //      let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    // For <CounterUs> we need to get from PAC rather than Timer
    //      let mut timer = dp.TIM2.counter_us(&clocks);
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(2.secs()).unwrap();

    // Generate an interrupt when the timer expires
    timer.listen(Event::Update);

    // Move initialised objects into globals
    cortex_m::interrupt::free(|cs| {
        G_LED.borrow(cs).replace(Some(led));
        G_KEY.borrow(cs).replace(Some(key));
        G_TIMER.borrow(cs).replace(Some(timer));
    });

    // Enable interrupts
    unsafe {
        pac::NVIC::unmask::<interrupt>(interrupt::EXTI0);
        pac::NVIC::unmask::<interrupt>(interrupt::TIM2);
    }

    loop {
        // Wait for interrupt (disables RTT)
        // cortex_m::asm::wfi();
    }
}

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(b) = G_KEY.borrow(cs).borrow_mut().as_mut() {
            b.clear_interrupt_pending_bit()
        }
        log::info!("KEY Pressed");
    });
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(led) = G_LED.borrow(cs).borrow_mut().as_mut() {
            led.toggle();
        }
        if let Some(t) = G_TIMER.borrow(cs).borrow_mut().as_mut() {
            let _ = t.wait();
        }
    });
}
