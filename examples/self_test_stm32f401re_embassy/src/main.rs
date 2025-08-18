#![no_std]
#![no_main]

use core::fmt::Write;
use core::fmt::{self, Display};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{self, USART2};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, Config as UsartConfig, DataBits, Parity, UartTx,
};
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use heapless::String;
use libm::fabsf;
use st_mems_bus::BusOperation;
use {defmt_rtt as _, panic_probe as _};

use lis2duxs12_rs::prelude::*;
use lis2duxs12_rs::{Error, I2CAddress, Lis2duxs12, ID};

#[repr(u8)]
#[derive(PartialEq, Debug)]
enum StTestType {
    StPos = 0,
    StNeg = 1,
}

#[repr(u8)]
#[derive(PartialEq)]
enum StResult {
    StPass = 1,
    StFail = 0,
}

impl Display for StTestType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StTestType::StPos => write!(f, "\nPOS"),
            StTestType::StNeg => write!(f, "NEG"),
        }
    }
}

const ST_RANGE_DEV_X_MIN: u16 = 50;
const ST_RANGE_DEV_X_MAX: u16 = 700;
const ST_RANGE_DEV_Y_MIN: u16 = 50;
const ST_RANGE_DEV_Y_MAX: u16 = 700;
const ST_RANGE_DEV_Z_MIN: u16 = 200;
const ST_RANGE_DEV_Z_MAX: u16 = 1200;

fn st_avg_5_samples<B, T>(
    sensor: &mut Lis2duxs12<B, T>,
    md: &Md,
    fmode: &FifoMode,
) -> Result<FifoData, Error<B::Error>>
where
    B: BusOperation,
    T: DelayNs,
{
    let mut fdata = FifoData::default();

    for _ in 0..5 {
        let tmp = sensor.fifo_data_get(md, fmode)?;

        for j in 0..3 {
            fdata.xl[0].mg[j] += tmp.xl[0].mg[j];
        }
    }

    for j in 0..3 {
        fdata.xl[0].mg[j] /= 5.0;
    }

    Ok(fdata)
}

#[defmt::panic_handler]
fn panic() -> ! {
    core::panic!("panic via `defmt::panic!`")
}

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut usart_config: UsartConfig = UsartConfig::default();
    usart_config.baudrate = 115200;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx: UartTx<_> = UartTx::new(p.USART2, p.PA2, NoDma, usart_config).unwrap();

    let i2c: I2c<_> = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        NoDma,
        NoDma,
        khz(100),
        I2cConfig::default(),
    );

    let mut delay = Delay;

    delay.delay_ms(5_u32);

    let mut msg: String<64> = String::new();

    let mut sensor = Lis2duxs12::new_i2c(i2c, I2CAddress::I2cAddH, delay.clone());

    // Exit deep power down
    // sensor.exit_deep_power_down().unwrap(); Only SPI

    // Check device ID
    match sensor.device_id_get() {
        Ok(id) => {
            if id != ID {
                loop {}
            }
        }
        Err(e) => {
            writeln!(&mut msg, "Error in reading id: {:?}", e).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }

    // Restore default configuration
    sensor.init_set(Init::Reset).unwrap();

    // Wait for reset to complete
    while sensor.status_get().unwrap().sw_reset == 1 {}

    /*
     * Accelerometer Self Test
     */
    loop {
        for test in [StTestType::StPos, StTestType::StNeg] {
            /*
             * 1. Set the device in soft power-down and wait 10ms.
             */
            let mut md = Md {
                fs: Fs::_8g,
                bw: Bw::OdrDiv2,
                odr: Odr::Off,
            };
            sensor.mode_set(&md).unwrap();
            delay.delay_ms(10_u32);

            /*
             * 2. Set the FIFO_EN bit in the CTRL4 (13h) register to 1.
             * 3. Set the XL_ONLY_FIFO bit in the FIFO_WTM (16h) register to 1.
             * 5. Set the FIFO_CTRL (15h) register to 00h to empty the FIFO
             */
            let mut fifo_mode = sensor.fifo_mode_get().unwrap();
            fifo_mode.operation = FifoOperation::BypassMode;
            fifo_mode.xl_only = 1;
            fifo_mode.store = Store::Fifo1x;
            let _ = sensor.fifo_mode_set(&fifo_mode);

            /*
             * 4. Set the ST_SIGN_X and ST_SIGN_Y bits in the CTRL3 (12h) register to 1
             *    and the ST_SIGN_Z bit in the WAKE_UP_DUR (1Dh) register to 0
             *    (i.e. 001 for POSITIVE. Instead, for NEGATIVE is 010).
             */
            if test == StTestType::StPos {
                sensor.self_test_sign_set(XlSelfTest::Positive).unwrap();
            } else {
                sensor.self_test_sign_set(XlSelfTest::Negative).unwrap();
            }

            /*
             * 6. Set ST[1:0] to "10"
             */
            sensor.self_test_start(2).unwrap();

            /*
             * 7. Set ODR = 200 hz, BW = ODR/2, FS = +/-8 g from the CTRL5 (14h) register
             *    and wait 50ms.
             */
            md.fs = Fs::_8g;
            md.bw = Bw::OdrDiv2;
            md.odr = Odr::_200hzLp;
            sensor.mode_set(&md).unwrap();
            delay.delay_ms(50_u32);

            /*
             * 8. Set the FIFO_CTRL (15h) register to 01h to start filling the FIFO.
             */
            let mut fifo_mode = sensor.fifo_mode_get().unwrap();
            fifo_mode.operation = FifoOperation::FifoMode;
            sensor.fifo_mode_set(&fifo_mode).unwrap();

            /*
             * 9. Read the first 5 samples from FIFO, compute the average for each
             *    axis, and save the result in OUT1.
             */
            let mut num = 0;
            while num < 5 {
                num = sensor.fifo_data_level_get().unwrap();
            }

            let fdata1 = st_avg_5_samples(&mut sensor, &md, &fifo_mode).unwrap();

            /*
             * 10. Set device in Power Down mode and wait 10 ms.
             */
            let md = Md {
                fs: Fs::_8g,
                bw: Bw::OdrDiv2,
                odr: Odr::Off,
            };
            sensor.mode_set(&md).unwrap();
            delay.delay_ms(10_u32);

            /*
             * 11. Set the FIFO_CTRL (15h) register to 00h to empty the FIFO.
             */
            let mut fifo_mode = sensor.fifo_mode_get().unwrap();
            fifo_mode.operation = FifoOperation::BypassMode;
            let _ = sensor.fifo_mode_set(&fifo_mode);

            /*
             * 12. Set ST[1:0] to "01"
             */
            let _ = sensor.self_test_start(1);

            /*
             * 13. Set ODR = 200 hz, BW = ODR/2, FS = +/-8 g from the CTRL5 (14h) register
             *     and wait 50ms.
             */
            let md = Md {
                fs: Fs::_8g,
                bw: Bw::OdrDiv2,
                odr: Odr::_200hzLp,
            };
            sensor.mode_set(&md).unwrap();
            delay.delay_ms(50_u32);

            /*
             * 14. Set the FIFO_CTRL (15h) register to 01h to start filling the FIFO
             *     and wait 25ms.
             */
            let mut fifo_mode = sensor.fifo_mode_get().unwrap();
            fifo_mode.operation = FifoOperation::FifoMode;
            let _ = sensor.fifo_mode_set(&fifo_mode);
            delay.delay_ms(25_u32);

            /*
             * 15. Read the first 5 samples from FIFO, compute the average for each
             *     axis, and save the result in OUT2.
             */
            num = 0;
            while num < 5 {
                num = sensor.fifo_data_level_get().unwrap();
            }

            let fdata2 = st_avg_5_samples(&mut sensor, &md, &fifo_mode).unwrap();

            /*
             * 16. Set device in Power Down mode and wait 10 ms.
             */
            let md = Md {
                fs: Fs::_8g,
                bw: Bw::OdrDiv2,
                odr: Odr::Off,
            };
            sensor.mode_set(&md).unwrap();
            delay.delay_ms(10_u32);

            /*
             * 17. Set the ST[1:0] bits in the SELF_TEST (32h) register to 00.
             */
            sensor.self_test_stop().unwrap();

            /*
             * 18. Self-test deviation is 2 * |OUT2 - OUT1|. Compute the value for
             * each axis and verify that it falls within the range provided in the
             * datasheet
             */
            let mut st_dev = FifoData::default();
            for i in 0..3 {
                st_dev.xl[0].mg[i] = 2.0 * fabsf(fdata2.xl[0].mg[i] - fdata1.xl[0].mg[i]);
            }
            /*
             * 19. Set device in Power Down mode
             */
            let md = Md {
                fs: Fs::_8g,
                bw: Bw::OdrDiv2,
                odr: Odr::Off,
            };
            sensor.mode_set(&md).unwrap();

            /* check if stdev falls into given ranges */
            let mut st_result = StResult::StFail;
            if (st_dev.xl[0].mg[0] >= ST_RANGE_DEV_X_MIN.into()
                && st_dev.xl[0].mg[0] <= ST_RANGE_DEV_X_MAX.into())
                && (st_dev.xl[0].mg[1] >= ST_RANGE_DEV_Y_MIN.into()
                    && st_dev.xl[0].mg[1] <= ST_RANGE_DEV_Y_MAX.into())
                && (st_dev.xl[0].mg[2] >= ST_RANGE_DEV_Z_MIN.into()
                    && st_dev.xl[0].mg[2] <= ST_RANGE_DEV_Z_MAX.into())
            {
                st_result = StResult::StPass;
            }

            if st_result == StResult::StPass {
                writeln!(&mut msg, "{} Self Test - PASS", test).unwrap();
            } else {
                writeln!(&mut msg, "{} Self Test - FAIL!!!!", test).unwrap();
            }

            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }
}
