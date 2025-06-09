#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use embedded_sss as _; // global logger + panicking-behavior + memory layout

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
    use defmt::debug;
    use display_interface_spi::SPIInterface;
    use embedded_graphics::{draw_target::DrawTarget, prelude::*};
    use ft6x06_rs::FT6x06;
    use hal::{
        gpio::{Output, Pin},
        hal::{
            digital::v2::{OutputPin, ToggleableOutputPin},
            spi,
        },
        pac,
        prelude::*,
        pwr::PwrExt,
        rcc::RccExt,
        spi::Spi,
        timer::Timer,
    };
    use ili9341::Ili9341;
    use stm32h7xx_hal::{self as hal, delay::Delay};

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        // red_led: Pin<'B', 14, Output>,
        // timer: Timer<pac::TIM2>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // TODO setup monotonic if used
        // let sysclk = { /* clock setup + returning sysclk as an u32 */ };
        // let token = rtic_monotonics::create_systick_token!();
        // rtic_monotonics::systick::Systick::new(cx.core.SYST, sysclk, token);
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
        let mut delay = Delay::new(cp.SYST, ccdr.clocks);

        let interface = SPIInterface::new(spi, dc, display_cs);
        let mut display = Ili9341::new(
            interface,
            rst,
            &mut delay,
            ili9341::Orientation::Portrait,
            ili9341::DisplaySize240x320,
        )
        .unwrap();

        display
            .clear(embedded_graphics::pixelcolor::Rgb565::BLACK)
            .unwrap();

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
}
