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
    let interrupt = Input::new(p.PB0, Pull::None);
    let mut interrupt = ExtiInput::new(interrupt, p.EXTI0);

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

    let wakeup_cfg = WakeupConfig {
        wake_dur: WakeDur::_0Odr,
        sleep_dur: 0,
        wake_ths_weight: 1,
        wake_ths: 2,
        wake_enable: WakeEnable::SleepOn,
        inact_odr: InactOdr::NoChange,
    };
    sensor.wakeup_config_set(wakeup_cfg).unwrap();

    // Configure interrupt pins
    let int1_route = PinIntRoute {
        wake_up: 1,
        ..Default::default()
    };
    sensor.pin_int1_route_set(&int1_route).unwrap();

    let int_mode = IntConfig {
        int_cfg: IntCfg::Latched,
        ..Default::default()
    };
    sensor.int_config_set(&int_mode).unwrap();

    // Set Output Data Rate
    let md = Md {
        fs: Fs::_2g,
        bw: Bw::OdrDiv2,
        odr: Odr::_200hzLp,
    };
    sensor.mode_set(&md).unwrap();

    loop {
        interrupt.wait_for_rising_edge().await;

        let status_all = sensor.all_sources_get().unwrap();
        if status_all.wake_up == 1 {
            writeln!(&mut msg, "WAKEUP event detected").unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }
}
