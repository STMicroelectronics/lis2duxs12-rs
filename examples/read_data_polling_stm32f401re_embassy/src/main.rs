#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
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
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use lis2duxs12_rs::{prelude::*, I2CAddress, Lis2duxs12, ID};

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
    // sensor.exit_deep_power_down().unwrap(); // Only SPI

    // Check device ID
    match sensor.device_id_get() {
        Ok(id) => {
            if id != ID {
                loop {}
            }
        }
        Err(e) => {
            writeln!(&mut msg, "Error in reading id: {:?}", e).unwrap();
            msg.clear();
        }
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
                        &mut msg,
                        "Acceleration [mg]: {:.2}\t{:.2}\t{:.2}",
                        data_xl.mg[0], data_xl.mg[1], data_xl.mg[2]
                    )
                    .unwrap();
                    let _ = tx.blocking_write(msg.as_bytes());
                    msg.clear();
                }
                Err(e) => {
                    writeln!(&mut msg, "Error in reading acceleration data: {:?}", e).unwrap();
                    let _ = tx.blocking_write(msg.as_bytes());
                    msg.clear();
                }
            }

            // Read temperature data
            match sensor.outt_data_get() {
                Ok(data_temp) => {
                    writeln!(&mut msg, "Temp [degC]: {:.2}", data_temp.heat.deg_c).unwrap();
                    let _ = tx.blocking_write(msg.as_bytes());
                    msg.clear();
                }
                Err(e) => {
                    writeln!(&mut msg, "Error in reading temperature data: {:?}", e).unwrap();
                    let _ = tx.blocking_write(msg.as_bytes());
                    msg.clear();
                }
            }
        }
        delay.delay_ms(1000_u32);
    }
}
