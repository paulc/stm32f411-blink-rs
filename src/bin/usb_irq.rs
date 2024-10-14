#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use heapless;
use panic_rtt_target as _;
use stm32f4xx_hal::{
    gpio::{Edge, Input, PA0},
    otg_fs::{UsbBus, USB},
    pac::{self, interrupt, Interrupt},
    prelude::*,
    timer::{CounterHz, Event, Timer},
};
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// USB Devices
type UsbDeviceType<'a> = UsbDevice<'a, UsbBus<USB>>;
type UsbSerialType<'a> = SerialPort<'a, UsbBus<USB>>;
static G_USB_DEVICE: Mutex<RefCell<Option<UsbDeviceType>>> = Mutex::new(RefCell::new(None));
static G_USB_SERIAL: Mutex<RefCell<Option<UsbSerialType>>> = Mutex::new(RefCell::new(None));

// Msg Buf
type UsbMsg = heapless::String<64>;
type UsbMsgBuf = heapless::Deque<UsbMsg, 16>;
static G_MSG_BUFFER: Mutex<RefCell<UsbMsgBuf>> = Mutex::new(RefCell::new(UsbMsgBuf::new()));

// USB TTY Input Buf
type TtyInputBuf = heapless::Vec<u8, 64>;
static G_TTY_INPUT_BUF: Mutex<RefCell<TtyInputBuf>> = Mutex::new(RefCell::new(TtyInputBuf::new()));

// Button
type Key = PA0<Input>;
static G_KEY: Mutex<RefCell<Option<Key>>> = Mutex::new(RefCell::new(None));

// Timer
type Tim = CounterHz<pac::TIM2>;
static G_TIMER: Mutex<RefCell<Option<Tim>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Static
    static mut USB_BUF: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

    // Initialise RTT
    rtt_log::init();
    log::info!("STARTING");

    // Syetem preipherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Need 48MHz PLL for USB
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(25.MHz())
        .sysclk(48.MHz())
        .require_pll48clk()
        .freeze();

    // Needed for interrupt handler
    let mut syscfg = dp.SYSCFG.constrain();
    let mut exti = dp.EXTI;

    let mut _delay = cp.SYST.delay(&clocks);

    let gpioa = dp.GPIOA.split();

    // KEY button
    let mut key = gpioa.pa0.into_pull_up_input();

    // Enable interrupt
    key.make_interrupt_source(&mut syscfg);
    key.trigger_on_edge(&mut exti, Edge::Falling);
    key.enable_interrupt(&mut exti);

    // TIMER
    let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer.start(10.Hz()).unwrap();

    // Generate an interrupt when the timer expires
    timer.listen(Event::Update);

    // Setup USB
    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &clocks,
    );

    // Need usb_bus to have static lifetime due to refs in IRQs
    USB_BUS.replace(UsbBus::new(usb, &mut USB_BUF[..]));

    // Move devices to static globals to allow access from IRQ
    cortex_m::interrupt::free(|cs| {
        // Button
        G_KEY.borrow(cs).replace(Some(key));

        // Timer
        G_TIMER.borrow(cs).replace(Some(timer));

        // USB
        if let Some(usb_bus) = USB_BUS.as_ref() {
            let serial = SerialPort::new(usb_bus);

            let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .device_class(USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("NA")
                    .product("STM32F411 BlackPill")
                    .serial_number("-blackpill")])
                .unwrap()
                .build();

            G_USB_SERIAL.borrow(cs).replace(Some(serial));
            G_USB_DEVICE.borrow(cs).replace(Some(usb_dev));
        }
    });

    // Unmask interrupts
    unsafe {
        NVIC::unmask(Interrupt::OTG_FS);
        NVIC::unmask(Interrupt::EXTI0);
        pac::NVIC::unmask::<interrupt>(interrupt::TIM2);
    }

    loop {}
}

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        // log::info!("EXTI0 IRQ");
        if let Some(b) = G_KEY.borrow(cs).borrow_mut().as_mut() {
            b.clear_interrupt_pending_bit()
        }
        let mut msg_buf = G_MSG_BUFFER.borrow(cs).borrow_mut();
        let _ = msg_buf.push_back(heapless::String::try_from("BUTTON\r\n").unwrap());
    });
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        let mut msg_buf = G_MSG_BUFFER.borrow(cs).borrow_mut();
        if !msg_buf.is_empty() {
            if let Some(serial) = G_USB_SERIAL.borrow(cs).borrow_mut().as_mut() {
                if let Some(msg) = msg_buf.pop_front() {
                    match serial.write(msg.as_bytes()) {
                        Ok(n) => {
                            log::info!("USB Write :: {} bytes", n);
                        }
                        Err(e) => {
                            log::info!("USB Write :: ERR {:?}", e);
                        }
                    }
                    let _ = serial.flush();
                }
            }
        }
    });
}

#[interrupt]
fn OTG_FS() {
    cortex_m::interrupt::free(|cs| {
        // log::info!("OTG_FS IRQ");
        if let Some(usb_dev) = G_USB_DEVICE.borrow(cs).borrow_mut().as_mut() {
            if let Some(serial) = G_USB_SERIAL.borrow(cs).borrow_mut().as_mut() {
                let mut input_buf = G_TTY_INPUT_BUF.borrow(cs).borrow_mut();
                let mut msg_buf = G_MSG_BUFFER.borrow(cs).borrow_mut();
                if usb_dev.poll(&mut [serial]) {
                    let mut buf = [0u8; 64];
                    match serial.read(&mut buf) {
                        Ok(n) => {
                            for i in 0..n {
                                match &buf[i] {
                                    b'\n' | b'\r' => {
                                        if let Ok(s) =
                                            heapless::String::from_utf8(input_buf.clone())
                                        {
                                            if s.len() > 0 {
                                                log::info!("SERIAL_IN: {}", s);
                                                let _ = msg_buf.push_back(
                                                    heapless::String::try_from("RX: >").unwrap(),
                                                );
                                                let _ = msg_buf.push_back(s);
                                                let _ = msg_buf.push_back(
                                                    heapless::String::try_from("<\r\n").unwrap(),
                                                );
                                            }
                                        }
                                        input_buf.clear();
                                    }
                                    _ => match input_buf.push(buf[i]) {
                                        Ok(_) => {}
                                        Err(_) => {
                                            if let Ok(s) =
                                                heapless::String::from_utf8(input_buf.clone())
                                            {
                                                log::info!("SERIAL_IN: {}", s);
                                                let _ = msg_buf.push_back(
                                                    heapless::String::try_from("RX: >").unwrap(),
                                                );
                                                let _ = msg_buf.push_back(s);
                                                let _ = msg_buf.push_back(
                                                    heapless::String::try_from("<\r\n").unwrap(),
                                                );
                                            }
                                            input_buf.clear();
                                        }
                                    },
                                }
                            }
                            log::info!("USB Read :: {:?}", &buf[0..n]);
                        }
                        Err(e) => {
                            log::info!("USB Read :: ERR {:?}", e);
                        }
                    }
                }
            }
        }
    });
}
