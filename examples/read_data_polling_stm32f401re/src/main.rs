#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::Write;
use cortex_m_rt::entry;
use lis2duxs12_rs::{prelude::*, I2CAddress, Lis2duxs12, ID};
use panic_halt as _;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::{config::Config, Serial},
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);
    let tim1 = dp.TIM1.delay_us(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let sda = gpiob.pb9.into_alternate().set_open_drain();

    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = Serial::tx(
        dp.USART2,
        tx_pin,
        Config::default().baudrate(115_200.bps()),
        &clocks,
    )
    .unwrap();

    delay.delay_ms(5);

    let mut sensor = Lis2duxs12::new_i2c(i2c, I2CAddress::I2cAddH, tim1);

    // Exit deep power down
    // sensor.exit_deep_power_down().unwrap(); // Only SPI

    // Check device ID
    match sensor.device_id_get() {
        Ok(id) => {
            if id != ID {
                loop {}
            }
        }
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

    // Restore default configuration
    sensor.init_set(Init::Reset).unwrap();

    // Wait for reset to complete
    while sensor.status_get().unwrap().sw_reset == 1 {}

    // Set BDU and IF_INC recommended for driver usage
    sensor.init_set(Init::SensorOnlyOn).unwrap();

    // Set Output Data Rate
    let md = Md {
        fs: Fs::_4g,
        bw: Bw::OdrDiv4,
        odr: Odr::_25hzLp,
    };
    sensor.mode_set(&md).unwrap();

    // Read samples in polling mode (no int)
    loop {
        // Read output only if new values are available
        if sensor.status_get().unwrap().drdy != 0 {
            // Read acceleration data
            match sensor.xl_data_get(&md) {
                Ok(data_xl) => {
                    writeln!(
                        tx,
                        "Acceleration [mg]: {:.2}\t{:.2}\t{:.2}",
                        data_xl.mg[0], data_xl.mg[1], data_xl.mg[2]
                    )
                    .unwrap();
                }
                Err(e) => writeln!(tx, "Error in reading acceleration data: {:?}", e).unwrap(),
            }

            // Read temperature data
            match sensor.outt_data_get() {
                Ok(data_temp) => {
                    writeln!(tx, "Temp [degC]: {:.2}", data_temp.heat.deg_c).unwrap();
                }
                Err(e) => writeln!(tx, "Error in reading temperature data: {:?}", e).unwrap(),
            }
        }
        delay.delay_ms(1000_u32);
    }
}
