#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use embassy_stm32::time::Hertz;
use panic_probe as _;

use embassy_stm32 as _; // memory layout

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

use embassy_stm32::timer::low_level::Timer;
use embedded_hal::delay::DelayNs;

pub struct TimerDelay<T>
where
    T: embassy_stm32::timer::CoreInstance,
{
    timer: Timer<'static, T>,
}

impl<T: embassy_stm32::timer::CoreInstance> TimerDelay<T> {
    pub fn new(timer: Timer<'static, T>) -> Self {
        Self { timer }
    }
}

impl<T: embassy_stm32::timer::CoreInstance> DelayNs for TimerDelay<T> {
    fn delay_ns(&mut self, ns: u32) {
        let frequency = 1_000_000_000 / ns; // This does floor division which is idea bc rounding
                                            // frequency down will only make the delay longer, not shorter, which is preferable.
        self.timer.stop();
        self.timer.set_frequency(Hertz::hz(frequency));
        self.timer.reset();
        self.timer.start();

        let reset_reg = self.timer.regs_core().sr();

        while !reset_reg.read().uif() {}

        reset_reg.modify(|reg| reg.set_uif(false));
    }
}
