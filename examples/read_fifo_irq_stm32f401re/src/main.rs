#![no_std]
#![no_main]

use panic_halt as _;

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use lis2duxs12_rs::{prelude::*, I2CAddress, Lis2duxs12, ID};
use stm32f4xx_hal::{
    gpio::{self, Edge, Input},
    i2c::{DutyCycle, I2c, Mode},
    pac::{self, interrupt},
    prelude::*,
    serial::{config::Config, Serial},
};
type IntPin = gpio::PB0<Input>;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));
static MEMS_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static NUM_FIFO_ENTRY: u8 = 33; // 32 samples + timestamp

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
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
    let mut int_pin = gpiob.pb0.into_input();
    // Configure Pin for Interrupts
    // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
    let mut syscfg = dp.SYSCFG.constrain();
    // 2) Make an interrupt source
    int_pin.make_interrupt_source(&mut syscfg);
    // 3) Make an interrupt source
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    // 4) Enable gpio interrupt
    int_pin.enable_interrupt(&mut dp.EXTI);

    // Enable the external interrupt in the NVIC by passing the interrupt number
    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    // Now that pin is configured, move pin into global context
    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

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
        // Wait for interrupt
        let mems_event = cortex_m::interrupt::free(|cs| {
            let flag = *MEMS_EVENT.borrow(cs).borrow();
            if flag {
                MEMS_EVENT.borrow(cs).replace(false);
            }
            flag
        });
        if !mems_event {
            continue;
        }

        // Read number of samples in FIFO
        let mut num = sensor.fifo_data_level_get().unwrap() as u16;
        let mut ts: u32;
        writeln!(tx, "-- {} in FIFO", num).unwrap();
        while num > 0 {
            let curr_entry = (NUM_FIFO_ENTRY as u16) - num;
            num -= 1;
            let fdata = sensor.fifo_data_get(&md, &fifo_mode).unwrap();
            match FifoSensorTag::try_from(fdata.tag).unwrap_or_default() {
                FifoSensorTag::XlOnly2xTag => {
                    writeln!(
                        tx,
                        "{:2}_0: Acceleration [0][mg]:\t{:4.2}\t{:4.2}\t{:4.2}",
                        curr_entry, fdata.xl[0].mg[0], fdata.xl[0].mg[1], fdata.xl[0].mg[2]
                    )
                    .unwrap();
                    writeln!(
                        tx,
                        "{:2}_1: Acceleration [1][mg]:\t{:4.2}\t{:4.2}\t{:4.2}",
                        curr_entry, fdata.xl[1].mg[0], fdata.xl[1].mg[1], fdata.xl[1].mg[2]
                    )
                    .unwrap();
                }
                FifoSensorTag::XlTempTag => {
                    if fifo_mode.xl_only == 0 {
                        writeln!(
                            tx,
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
                            tx,
                            "{:2}: Acceleration [0][mg]:{:4.2}\t{:4.2}\t{:4.2}",
                            curr_entry, fdata.xl[0].mg[0], fdata.xl[0].mg[1], fdata.xl[0].mg[2]
                        )
                        .unwrap();
                    }
                }
                FifoSensorTag::TimestampTag => {
                    ts = fdata.cfg_chg.timestamp / 100;
                    writeln!(tx, "Timestamp:\t{} ms", ts).unwrap();
                }
                _ => {
                    writeln!(
                        tx,
                        "unknown TAG ({:#02x})",
                        FifoSensorTag::try_from(fdata.tag).unwrap_or_default() as u8
                    )
                    .unwrap();
                }
            }
        }

        writeln!(tx, "").unwrap();
    }
}

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        if int_pin.as_mut().unwrap().check_interrupt() {
            int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
        }
        MEMS_EVENT.borrow(cs).replace(true);
    })
}
