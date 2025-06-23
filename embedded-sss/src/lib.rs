#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use panic_probe as _;

use stm32h7xx_hal as _; // memory layout

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

use embedded_hal::i2c::{self, I2c, Operation, SevenBitAddress};

/// I2C0 hardware peripheral which supports both 7-bit and 10-bit addressing.
pub struct Eh1I2cWrapper<I2C> {
    inner: I2C,
}

#[derive(Debug)]
pub struct Eh1I2cError(pub stm32h7xx_hal::i2c::Error);

impl<I2C> Eh1I2cWrapper<I2C> {
    fn inner(&self) -> &I2C {
        &self.inner
    }

    fn inner_mut(&mut self) -> &I2C {
        &mut self.inner
    }

    pub fn new(i2c: I2C) -> Self {
        Self { inner: i2c }
    }
}

impl i2c::Error for Eh1I2cError {
    fn kind(&self) -> i2c::ErrorKind {
        use i2c::{ErrorKind, NoAcknowledgeSource};
        use stm32h7xx_hal::i2c::Error;
        match self.0 {
            Error::Bus => ErrorKind::Bus,
            Error::Arbitration => ErrorKind::ArbitrationLoss,
            Error::NotAcknowledge => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            _ => ErrorKind::Other,
        }
    }
}

impl<I2C> i2c::ErrorType for Eh1I2cWrapper<I2C>
where
    I2C: embedded_hal_02::blocking::i2c::Write<Error = stm32h7xx_hal::i2c::Error>
        + embedded_hal_02::blocking::i2c::Read<Error = stm32h7xx_hal::i2c::Error>
        + embedded_hal_02::blocking::i2c::WriteRead<Error = stm32h7xx_hal::i2c::Error>,
{
    type Error = Eh1I2cError;
}

impl From<stm32h7xx_hal::i2c::Error> for Eh1I2cError {
    fn from(e: stm32h7xx_hal::i2c::Error) -> Self {
        Self(e)
    }
}

impl<I2C> I2c<SevenBitAddress> for Eh1I2cWrapper<I2C>
where
    I2C: embedded_hal_02::blocking::i2c::Write<Error = stm32h7xx_hal::i2c::Error>
        + embedded_hal_02::blocking::i2c::Read<Error = stm32h7xx_hal::i2c::Error>
        + embedded_hal_02::blocking::i2c::WriteRead<Error = stm32h7xx_hal::i2c::Error>,
{
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                Operation::Read(buffer) => self.inner.read(address, buffer)?,
                Operation::Write(buffer) => self.inner.write(address, buffer)?,
            };
        }
        Ok(())
    }
}
