#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use embedded_sss as _; // global logger + panicking-behavior + memory layout

rtic_monotonics::systick_monotonic!(Mono, 1_000);

#[rtic::app(
    device = stm32h7xx_hal::pac,
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
    use embedded_graphics::{draw_target::DrawTarget, prelude::*};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use embedded_hal_compat::ForwardCompat;
    use embedded_sss::Eh1I2cWrapper;
    use ft6x06_rs::{ControlMode, FT6x06, InterruptMode};
    use ili9341::Ili9341;
    use rtic_monotonics::Monotonic;
    use stm32h7xx_hal::{
        self as hal, delay::DelayFromCountDownTimer, pac, prelude::*, pwr::PwrExt, rcc::RccExt, spi,
    };

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        ft6x06: FT6x06<Eh1I2cWrapper<stm32h7xx_hal::i2c::I2c<pac::I2C1>>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");

        debug!("No Periphs taken");
        let cp = cx.core;
        let dp = cx.device;
        // Hold CM4 core in reset to avoid conflicts
        debug!("Periphs taken");
        let pwr = dp.PWR.constrain();
        debug!("Power constrained");
        let pwrcfg = pwr.smps().freeze();
        debug!("Power frozen");
        match pwrcfg.vos() {
            hal::pwr::VoltageScale::Scale0 => debug!("Scale0"),
            hal::pwr::VoltageScale::Scale1 => debug!("Scale1"),
            hal::pwr::VoltageScale::Scale2 => debug!("Scale2"),
            hal::pwr::VoltageScale::Scale3 => debug!("Scale3"),
        }

        let rcc = dp.RCC.constrain();
        debug!("Clocks constrained");
        let ccdr = rcc
            .sys_ck(400.MHz()) // I'm not quite sure why I can't set these to 480 and 120
            // respectively even in VOS0
            .pclk1(100.MHz())
            .pll1_q_ck(32.MHz())
            .freeze(pwrcfg, &dp.SYSCFG);
        debug!("Clocks set");
        debug!(
            "sys_ck: {} pclk1: {}",
            ccdr.clocks.sys_ck().to_MHz(),
            ccdr.clocks.pclk1().to_MHz()
        );

        Mono::start(cp.SYST, ccdr.clocks.sys_ck().to_Hz());

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

        let sck = gpioa.pa5.into_alternate::<5>(); // SPI1 is alternate function 5
        let miso = gpioa.pa6.into_alternate::<5>();
        let mosi = gpiob.pb5.into_alternate::<5>();

        let spi = dp.SPI1.spi(
            (sck, miso, mosi),
            spi::MODE_0,
            16.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );

        let display_cs = gpiod.pd14.into_push_pull_output();
        let dc = gpiod.pd15.into_push_pull_output();
        let rst = gpiob.pb1.into_push_pull_output();
        let spi_device =
            ExclusiveDevice::new_no_delay(spi.forward(), display_cs.forward()).unwrap();
        let timer2 = dp.TIM2.timer(1.kHz(), ccdr.peripheral.TIM2, &ccdr.clocks);
        let delay = DelayFromCountDownTimer::new(timer2);

        let interface = SPIInterface::new(spi_device, dc.forward());
        let mut display = Ili9341::new(
            interface,
            rst.forward(),
            &mut delay.forward(),
            ili9341::Orientation::Portrait,
            ili9341::DisplaySize240x320,
        )
        .unwrap();

        display
            .clear(embedded_graphics::pixelcolor::Rgb565::BLACK)
            .unwrap();

        let scl = gpiob.pb8.into_alternate_open_drain(); // I2C1 is AF 4
        let sda = gpiob.pb9.into_alternate_open_drain();
        let i2c = dp
            .I2C1
            .i2c((scl, sda), 1.MHz(), ccdr.peripheral.I2C1, &ccdr.clocks);
        let wrapped_i2c = Eh1I2cWrapper::new(i2c);

        let mut dev = FT6x06::new(wrapped_i2c);

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

        // Get the latest touch data. Usually after receiving an interrupt from the device.

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
    async fn poll_touch(ctx: poll_touch::Context) {
        loop {
            debug!("Getting touch event");
            let touch_event = ctx.local.ft6x06.get_touch_event().unwrap();
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
