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
    // No longer a TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    //
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    //
    // From greg: These are some probably unused interrupts for weird signal processing stuff
    dispatchers = [DFSDM1_FLT0, DFSDM1_FLT1, DFSDM1_FLT2, DFSDM1_FLT3],
    // More stuff in here with serial audio stuff
    // dispatchers = [DFSDM1_FLT0, DFSDM1_FLT1, DFSDM1_FLT2, DFSDM1_FLT3, SPDIF, SAI1, SAI2, SAI3, SAI4]
    peripherals = true
)]
mod app {
    use defmt::{debug, info};
    use display_interface_spi::SPIInterface;
    use embassy_stm32::{self as hal, gpio, i2c, rcc, spi, time::mhz, timer};
    use embedded_graphics::{draw_target::DrawTarget, prelude::*};
    use embedded_hal::delay::DelayNs;
    use embedded_hal_bus::spi::ExclusiveDevice;
    use embedded_sss::TimerDelay;
    use ft6x06_rs::{ControlMode, FT6x06, InterruptMode};
    use fugit::ExtU32;
    use ili9341::Ili9341;
    use rtic_monotonics::Monotonic;

    use crate::{Mono, SHARED_DATA};

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        ft6x06: FT6x06<i2c::I2c<'static, hal::mode::Blocking>>,
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
        // Set the system clock source to PLL1
        config.rcc.sys = rcc::Sysclk::PLL1_P;
        // Divide some peripheral prescalers to keep them within limits
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2; // HPRE Prescaler
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV2; // D2PRE1
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2; // D2PRE2
        config.rcc.apb3_pre = rcc::APBPrescaler::DIV2; // D1PRE
        config.rcc.apb4_pre = rcc::APBPrescaler::DIV2; // D3PRE

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
        let rst = gpio::Output::new(p.PB1, gpio::Level::Low, gpio::Speed::High);
        let spi_device = ExclusiveDevice::new_no_delay(spi, display_cs).unwrap();
        let timer2 = timer::low_level::Timer::new(p.TIM2);
        let mut delay = TimerDelay::new(timer2);

        let interface = SPIInterface::new(spi_device, dc);
        let mut display = Ili9341::new(
            interface,
            rst,
            &mut delay,
            ili9341::Orientation::Portrait,
            ili9341::DisplaySize240x320,
        )
        .unwrap();
        debug!("Display Created");

        display
            .clear(embedded_graphics::pixelcolor::Rgb565::BLACK)
            .unwrap();
        debug!("Display Cleared");

        let i2c = i2c::I2c::new_blocking(p.I2C1, p.PB8, p.PB9, mhz(1), i2c::Config::default());
        let mut dev = FT6x06::new(i2c);

        // Configure the device.
        dev.set_interrupt_mode(InterruptMode::Trigger).unwrap();
        dev.set_control_mode(ControlMode::MonitorIdle).unwrap();
        dev.set_active_idle_timeout(10).unwrap();
        dev.set_report_rates(60, 25).unwrap();

        // Read the device configuration.
        let interrupt_mode = dev.get_interrupt_mode().unwrap();
        let control_mode = dev.get_control_mode().unwrap();
        let active_idle_timeout = dev.get_active_idle_timeout().unwrap();
        let (active_rate, monitor_rate) = dev.get_report_rates().unwrap();

        info!("Irq Mode: {}", interrupt_mode);
        info!("Ctrl Mode: {}", control_mode);
        info!("Active Idle Timeout: {}", active_idle_timeout);
        info!("Active Rate: {}", active_rate);
        info!("Monitor Rate: {}", monitor_rate);

        poll_touch::spawn().unwrap();
        (Shared {}, Local { ft6x06: dev })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        info!("idle");

        loop {
            continue;
        }
    }

    #[task(priority = 1, local = [ft6x06])]
    async fn poll_touch(cx: poll_touch::Context) {
        loop {
            debug!("Getting touch event");
            let touch_event = cx.local.ft6x06.get_touch_event().unwrap();
            if let Some(event) = touch_event {
                info!(
                    "Touch detected at: x: {}, y: {}",
                    event.primary_point.x, event.primary_point.y
                );
            }
            Mono::delay(50.millis()).await;
        }
    }
}
