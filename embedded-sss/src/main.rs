#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use core::mem::MaybeUninit;

use embedded_sss as _; // global logger + panicking-behavior + memory layout

rtic_monotonics::systick_monotonic!(Mono, 1_000);

#[link_section = ".axisram"]
static SHARED_DATA: MaybeUninit<embassy_stm32::SharedData> = MaybeUninit::uninit();

#[rtic::app(
    device = embassy_stm32,
    dispatchers = [DFSDM1_FLT0, DFSDM1_FLT1, DFSDM1_FLT2, DFSDM1_FLT3],
    peripherals = true
)]
mod app {
    use defmt::{debug, info};
    use display_interface_spi::SPIInterface;
    use embassy_stm32::{self as hal, gpio, i2c, mode, rcc, spi, time::mhz, timer};
    use embedded_graphics::{
        draw_target::DrawTarget,
        image::{Image, ImageRaw},
        pixelcolor::Rgb565,
        prelude::*,
    };
    use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
    use embedded_sss::TimerDelay;
    use ft6x06_rs::{ControlMode, FT6x06, InterruptMode};
    use fugit::ExtU32;
    use ili9341::Ili9341;
    use rtic_monotonics::Monotonic;
    use tinybmp::Bmp;

    use crate::{Mono, SHARED_DATA};

    type DisplaySPIInterface = SPIInterface<
        // GPIO pin is Chip Select
        ExclusiveDevice<spi::Spi<'static, mode::Blocking>, gpio::Output<'static>, NoDelay>,
        gpio::Output<'static>, // Data/Command
    >;

    /// gpio pin for the display reset
    type DisplayReset = gpio::Output<'static>;

    // Shared resources go here
    #[shared]
    struct Shared {
        display: Ili9341<DisplaySPIInterface, DisplayReset>,
    }

    // Local resources go here
    #[local]
    struct Local {
        touch_screen: FT6x06<i2c::I2c<'static, mode::Blocking>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");

        let cp = cx.core;

        let mut config = hal::Config::default();
        config.rcc.pll1 = Some(rcc::Pll {
            source: rcc::PllSource::HSI,    // 64 MHz -> DIVM1
            prediv: rcc::PllPreDiv::DIV4,   // DIVM1 = 4: 16 MHz -> DIVN1
            mul: rcc::PllMul::MUL60,        // DIVN1 = 60: 960 MHz -> DIVP1 + DIVQ1 + DIVR1
            divp: Some(rcc::PllDiv::DIV2),  // DIVP1 = 2: 480 MHz -> System clock + more
            divq: Some(rcc::PllDiv::DIV16), // DIVQ1 = 16: 60 MHz -> SPI1 + more
            divr: None,                     // Disabled
        });
        // Allow higher clock speed
        // (This isn't technically necessary since Scale0 is the default
        config.rcc.voltage_scale = rcc::VoltageScale::Scale0;
        // Set the system clock source to PLL1
        config.rcc.sys = rcc::Sysclk::PLL1_P;
        // Divide some peripheral prescalers to keep them within limits
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2; // HPRE Prescaler
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV2; // D2PRE1
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2; // D2PRE2
        config.rcc.apb3_pre = rcc::APBPrescaler::DIV2; // D1PRE
        config.rcc.apb4_pre = rcc::APBPrescaler::DIV2; // D3PRE
        config.rcc.supply_config = rcc::SupplyConfig::DirectSMPS; // THIS MAKES EVERYTHING WORK!

        // USB configuration
        // USB has to be clocked to 48MHz so simply use HSI48
        config.rcc.mux.usbsel = rcc::mux::Usbsel::HSI48;
        // This is required when using HSI48
        config.rcc.hsi48 = Some(rcc::Hsi48Config {
            sync_from_usb: true,
        });

        debug!("Initializing HAL...");
        let p = hal::init_primary(config, &SHARED_DATA);
        debug!("HAL Initialized");

        Mono::start(cp.SYST, 480_000_000); // 480 MHz System Clock
        debug!("Monotonic Started");

        let mut spi_config = spi::Config::default();
        spi_config.frequency = mhz(16);
        let spi = spi::Spi::new_blocking(p.SPI1, p.PA5, p.PB5, p.PA6, spi_config);

        let display_cs = gpio::Output::new(p.PD14, gpio::Level::Low, gpio::Speed::High);
        let dc = gpio::Output::new(p.PD15, gpio::Level::Low, gpio::Speed::High);
        let rst = gpio::Output::new(p.PB1, gpio::Level::Low, gpio::Speed::High); // Im not actually
                                                                                 // convinced this is connected, I have no clue why I made this PB1 which is on A3???
        let spi_device = ExclusiveDevice::new_no_delay(spi, display_cs).unwrap();
        let timer2 = timer::low_level::Timer::new(p.TIM2);
        let mut delay = TimerDelay::new(timer2);

        let interface = SPIInterface::new(spi_device, dc);
        let mut display = Ili9341::new(
            interface,
            rst,
            &mut delay,
            ili9341::Orientation::PortraitFlipped,
            ili9341::DisplaySize240x320,
        )
        .unwrap();
        debug!("Display Created");

        display
            .clear(embedded_graphics::pixelcolor::Rgb565::WHITE)
            .unwrap();
        debug!("Display Cleared");

        let bmp = Bmp::from_slice(include_bytes!("../image.bmp")).unwrap();
        let image = Image::new(&bmp, Point::zero());
        image.draw(&mut display).unwrap();

        let i2c = i2c::I2c::new_blocking(p.I2C1, p.PB8, p.PB9, mhz(1), i2c::Config::default());
        let mut touch_screen = FT6x06::new(i2c);

        // Configure the device.
        touch_screen
            .set_interrupt_mode(InterruptMode::Trigger)
            .unwrap();
        touch_screen
            .set_control_mode(ControlMode::MonitorIdle)
            .unwrap();
        touch_screen.set_active_idle_timeout(10).unwrap();
        touch_screen.set_report_rates(60, 25).unwrap();

        // Read the device configuration.
        let interrupt_mode = touch_screen.get_interrupt_mode().unwrap();
        let control_mode = touch_screen.get_control_mode().unwrap();
        let active_idle_timeout = touch_screen.get_active_idle_timeout().unwrap();
        let (active_rate, monitor_rate) = touch_screen.get_report_rates().unwrap();

        info!("Irq Mode: {}", interrupt_mode);
        info!("Ctrl Mode: {}", control_mode);
        info!("Active Idle Timeout: {}", active_idle_timeout);
        info!("Active Rate: {}", active_rate);
        info!("Monitor Rate: {}", monitor_rate);

        poll_touch::spawn().unwrap();
        (Shared { display }, Local { touch_screen })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        info!("idle");

        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(priority = 1, local = [touch_screen], shared = [display])]
    async fn poll_touch(mut cx: poll_touch::Context) {
        loop {
            debug!("Getting touch event");
            let touch_event = cx.local.touch_screen.get_touch_event().unwrap();
            // Extract x and y coordinates, or continue if there is no touch
            let (x, y) = match touch_event {
                Some(event) => (event.primary_point.x, event.primary_point.y),
                None => {
                    // Delay and yield execution if there is no touch
                    // This makes sure the loop doesn't delay or yield whenever there is currently
                    // a touch.
                    Mono::delay(50.millis()).await;
                    continue;
                }
            };
            info!("Touch detected at: x: {}, y: {}", x, y);
            cx.shared.display.lock(|display| {
                display
                    .draw_raw_slice(
                        x - 1,
                        y - 1,
                        x + 1,
                        y + 1,
                        &[Rgb565::BLACK.into_storage(); 9],
                    )
                    .unwrap();
            });
        }
    }
}
