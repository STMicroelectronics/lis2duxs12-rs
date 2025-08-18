#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull};
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

    // Configure the interrupt pin (if needed) and obtain handler.
    // On the Nucleo FR401 the interrupt pin is connected to pin PB0.
    let interrupt = Input::new(p.PA10, Pull::None);
    let mut interrupt = ExtiInput::new(interrupt, p.EXTI10);

    delay.delay_ms(5_u32);

    let mut msg: String<64> = String::new();

    let mut sensor = Lis2duxs12::new_i2c(i2c, I2CAddress::I2cAddL, delay.clone());

    // Exit deep power down
    // sensor.exit_deep_power_down().unwrap(); // Only SPI

    // Check device ID
    match sensor.device_id_get() {
        Ok(id) => {
            if id != ID {
                writeln!(&mut msg, "Mismatch id: {:?}", id).unwrap();
                let _ = tx.blocking_write(msg.as_bytes());
                msg.clear();
                loop {}
            }
        }
        Err(e) => {
            writeln!(&mut msg, "Error in reading id: {:?}", e).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }

    // Exit deep power down
    sensor.exit_deep_power_down().unwrap();

    // Restore default configuration
    sensor.init_set(Init::Reset).unwrap();

    // Wait for reset to complete
    while sensor.status_get().unwrap().sw_reset == 1 {}

    // Set BDU and IF_INC recommended for driver usage
    sensor.init_set(Init::SensorOnlyOn).unwrap();

    let int_route = PinIntRoute {
        drdy: 1,
        int_on_res: 1,
        ..Default::default()
    };
    sensor.pin_int1_route_set(&int_route).unwrap();

    // Set Output Data Rate
    let md = Md {
        fs: Fs::_4g,
        bw: Bw::OdrDiv4,
        odr: Odr::_25hzLp,
    };
    sensor.mode_set(&md).unwrap();

    let qvar_mode = AhQvarMode {
        ah_qvar_en: 1,
        ah_qvar_zin: AhQvarZin::_520mohm,
        ah_qvar_gain: AhQvarGain::_05,
        ..Default::default()
    };
    sensor.ah_qvar_mode_set(&qvar_mode).unwrap();

    loop {
        interrupt.wait_for_rising_edge().await;
        let status = sensor.status_get().unwrap();

        if status.drdy == 1 {
            let data_qvar = sensor.ah_qvar_data_get().unwrap();
            let data_xl = sensor.xl_data_get(&md).unwrap();

            writeln!(
                &mut msg,
                "Acceleration [mg]:{:4.2}\t{:4.2}\t{:4.2} - QVAR [LSB]: {}",
                data_xl.mg[0], data_xl.mg[1], data_xl.mg[2], data_qvar.raw
            )
            .unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }
}
