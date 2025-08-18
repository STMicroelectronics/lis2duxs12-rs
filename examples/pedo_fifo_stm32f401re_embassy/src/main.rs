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

static NUM_FIFO_ENTRY: u8 = 8;

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
    while sensor.status_get().unwrap().sw_reset == 1 {}
    // Enable of Embedded Function
    sensor.init_set(Init::SensorEmbFuncOn).unwrap();

    delay.delay_ms(10_u32);

    sensor
        .embedded_int_cfg_set(EmbeddedIntConfig::Latched)
        .unwrap();
    sensor.stpcnt_debounce_set(4).unwrap();

    let stpcnt_mode = StpcntMode {
        false_step_rej: 0,
        step_counter_enable: 1,
        step_counter_in_fifo: 1,
    };
    sensor.stpcnt_mode_set(&stpcnt_mode).unwrap();
    sensor.stpcnt_rst_step_set().unwrap();

    // Set FIFO mode
    let fifo_mode = FifoMode {
        operation: FifoOperation::StreamMode,
        store: Store::Fifo2x,
        watermark: NUM_FIFO_ENTRY,
        xl_only: 0,
        cfg_change_in_fifo: 0,
        fifo_event: FifoEvent::Wtm,
        batch: Batch {
            dec_ts: DecTs::_1,
            bdr_xl: BdrXl::OdrOff,
        },
    };
    sensor.fifo_mode_set(&fifo_mode).unwrap();
    sensor.timestamp_set(1).unwrap();

    // Configure interrupt pins
    let int1_route = PinIntRoute {
        fifo_th: 1,
        ..Default::default()
    };
    sensor.pin_int1_route_set(&int1_route).unwrap();

    // Set Output Data Rate
    let md = Md {
        fs: Fs::_4g,
        bw: Bw::OdrDiv4,
        odr: Odr::_25hzLp,
    };
    sensor.mode_set(&md).unwrap();

    loop {
        interrupt.wait_for_rising_edge().await;

        let wmflag = sensor.fifo_wtm_flag_get().unwrap();

        if wmflag > 0 {
            let num = sensor.fifo_data_level_get().unwrap();

            writeln!(&mut msg, "-- {} in FIFO", num).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();

            let fdata = sensor.fifo_data_get(&md, &fifo_mode).unwrap();

            if let FifoSensorTag::StepCounterTag =
                FifoSensorTag::try_from(fdata.tag).unwrap_or_default()
            {
                let ts = fdata.pedo.timestamp as f32 / 100f32;
                let steps = fdata.pedo.steps;

                writeln!(&mut msg, "Steps: {:03} ({} ms)", steps, ts).unwrap();
                let _ = tx.blocking_write(msg.as_bytes());
                msg.clear();
            }
        }
    }
}
