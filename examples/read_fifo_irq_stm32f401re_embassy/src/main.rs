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

static NUM_FIFO_ENTRY: u8 = 33; // 32 samples + timestamp

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

    // Set FIFO watermark to 32 sample(s)
    let fifo_mode = FifoMode {
        store: Store::Fifo1x,
        xl_only: 0,
        watermark: NUM_FIFO_ENTRY,
        fifo_event: FifoEvent::Wtm,
        operation: FifoOperation::StreamMode,
        batch: Batch {
            dec_ts: DecTs::_32,
            bdr_xl: BdrXl::Odr,
        },
        cfg_change_in_fifo: 0,
    };
    sensor.fifo_mode_set(&fifo_mode).unwrap();
    sensor.timestamp_set(1).unwrap();

    // Configure interrupt pins
    let int1_route = PinIntRoute {
        int_on_res: 1,
        boot: 1,
        fifo_th: 1,
        fifo_ovr: 1,
        fifo_full: 1,
        ..Default::default()
    };
    sensor.pin_int1_route_set(&int1_route).unwrap();
    //sensor.pin_int2_route_set(&int1_route).unwrap();

    // Set Output Data Rate
    let md = Md {
        fs: Fs::_4g,
        bw: Bw::OdrDiv4,
        odr: Odr::_25hzLp,
    };
    sensor.mode_set(&md).unwrap();

    loop {
        interrupt.wait_for_rising_edge().await;

        // Read number of samples in FIFO
        let mut num = sensor.fifo_data_level_get().unwrap() as u16;
        let mut ts: u32;
        writeln!(&mut msg, "-- {} in FIFO", num).unwrap();
        while num > 0 {
            let curr_entry = (NUM_FIFO_ENTRY as u16) - num;
            num -= 1;
            let fdata = sensor.fifo_data_get(&md, &fifo_mode).unwrap();
            match FifoSensorTag::try_from(fdata.tag).unwrap_or_default() {
                FifoSensorTag::XlOnly2xTag => {
                    writeln!(
                        &mut msg,
                        "{:2}_0: Acceleration [0][mg]:\t{:4.2}\t{:4.2}\t{:4.2}",
                        curr_entry, fdata.xl[0].mg[0], fdata.xl[0].mg[1], fdata.xl[0].mg[2]
                    )
                    .unwrap();
                    writeln!(
                        &mut msg,
                        "{:2}_1: Acceleration [1][mg]:\t{:4.2}\t{:4.2}\t{:4.2}",
                        curr_entry, fdata.xl[1].mg[0], fdata.xl[1].mg[1], fdata.xl[1].mg[2]
                    )
                    .unwrap();
                }
                FifoSensorTag::XlTempTag => {
                    if fifo_mode.xl_only == 0 {
                        writeln!(
                            &mut msg,
                            "{:2}: Acceleration [0][mg]:{:4.2}\t{:4.2}\t{:4.2}\tTemp[degC]:{:3.2}",
                            curr_entry,
                            fdata.xl[0].mg[0],
                            fdata.xl[0].mg[1],
                            fdata.xl[0].mg[2],
                            fdata.heat.deg_c
                        )
                        .unwrap();
                    } else {
                        writeln!(
                            &mut msg,
                            "{:2}: Acceleration [0][mg]:{:4.2}\t{:4.2}\t{:4.2}",
                            curr_entry, fdata.xl[0].mg[0], fdata.xl[0].mg[1], fdata.xl[0].mg[2]
                        )
                        .unwrap();
                    }
                }
                FifoSensorTag::TimestampTag => {
                    ts = fdata.cfg_chg.timestamp / 100;
                    writeln!(&mut msg, "Timestamp:\t{} ms", ts).unwrap();
                }
                _ => {
                    writeln!(
                        &mut msg,
                        "unknown TAG ({:#02x})",
                        FifoSensorTag::try_from(fdata.tag).unwrap_or_default() as u8
                    )
                    .unwrap();
                }
            }
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }

        writeln!(&mut msg, "").unwrap();
    }
}
