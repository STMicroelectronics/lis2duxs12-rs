#![no_std]
#![no_main]

mod glance_detection;

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

use glance_detection::GLANCE_DETECTION;
use lis2duxs12_rs::{prelude::*, I2CAddress, Lis2duxs12, ID};
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

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

    // Start Machine Learning Core configuration

    for ucf_entry in GLANCE_DETECTION {
        match ucf_entry.op {
            MemsUcfOp::Delay => {
                delay.delay_ms(ucf_entry.data as u32);
            }
            MemsUcfOp::Write => {
                sensor
                    .write_to_register(ucf_entry.address, &[ucf_entry.data])
                    .unwrap();
            }
            _ => {}
        }
    }

    loop {
        interrupt.wait_for_rising_edge().await;
        let status = sensor.fsm_status_get().unwrap();

        if status.is_fsm1() == 1 {
            let fsm_out = sensor.fsm_out_get().unwrap();
            let fsm_out = fsm_out[0]; // the ucf store the fsm in the first index

            if fsm_out == 0x20 {
                // glance event catched

                writeln!(&mut msg, "GLANCE event").unwrap();
            }
            if fsm_out == 0x08 {
                // deglance event catched

                writeln!(&mut msg, "DEGLANCE event").unwrap();
            }
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }
}
