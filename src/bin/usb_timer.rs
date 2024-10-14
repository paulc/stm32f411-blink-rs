#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use stm32f4xx_hal::{
    otg_fs::{UsbBus, USB},
    pac::{self, interrupt, Interrupt},
    prelude::*,
};
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

type UsbDeviceType<'a> = UsbDevice<'a, UsbBus<USB>>;
type UsbSerialType<'a> = SerialPort<'a, UsbBus<USB>>;

static G_USB_DEVICE: Mutex<RefCell<Option<UsbDeviceType>>> = Mutex::new(RefCell::new(None));
static G_USB_SERIAL: Mutex<RefCell<Option<UsbSerialType>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Static
    static mut USB_BUF: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

    // Initialise RTT
    rtt_log::init();

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

    let mut _delay = cp.SYST.delay(&clocks);

    let gpioa = dp.GPIOA.split();

    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &clocks,
    );

    USB_BUS.replace(UsbBus::new(usb, &mut USB_BUF[..]));

    cortex_m::interrupt::free(|cs| {
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

    unsafe {
        NVIC::unmask(Interrupt::OTG_FS);
    }

    loop {}
}

#[interrupt]
fn OTG_FS() {
    cortex_m::interrupt::free(|cs| {
        if let Some(usb_dev) = G_USB_DEVICE.borrow(cs).borrow_mut().as_mut() {
            if let Some(serial) = G_USB_SERIAL.borrow(cs).borrow_mut().as_mut() {
                if usb_dev.poll(&mut [serial]) {
                    let _ = serial.write("Hello\r\n".as_bytes());
                }
            }
        }
    });
}
