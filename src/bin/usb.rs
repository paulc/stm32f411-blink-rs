#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use stm32f4xx_hal::{
    otg_fs::{UsbBus, USB},
    pac,
    prelude::*,
};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut USB_BUF: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
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

    let mut delay = cp.SYST.delay(&clocks);

    let gpioa = dp.GPIOA.split();

    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &clocks,
    );

    let usb_bus = unsafe {
        let usb_buf: &'static mut [u32] = &mut USB_BUF[..];
        UsbBus::new(usb, usb_buf)
    };

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .device_class(USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake Company")
            .product("Product")
            .serial_number("TEST")])
        .unwrap()
        .build();

    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let _ = serial.write("Hello\r\n".as_bytes());
        }
        // need to call usb_dev.poll at least every 10ms
        delay.delay_ms(5);
    }
}
