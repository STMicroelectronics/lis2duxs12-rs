#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use lis2duxs12_rs::{prelude::*, I2CAddress, Lis2duxs12, ID};
use panic_halt as _;
use stm32f4xx_hal::{
    gpio,
    gpio::{Edge, Input},
    i2c::{DutyCycle, I2c, Mode},
    interrupt, pac,
    prelude::*,
    serial::{config::Config, Serial},
};
type IntPin = gpio::PB0<Input>;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));
static MEMS_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

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

    // setup interrupt
    let mut int_pin = gpiob.pb0.into_pull_up_input();
    let mut syscfg = dp.SYSCFG.constrain();
    int_pin.make_interrupt_source(&mut syscfg);
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    int_pin.enable_interrupt(&mut dp.EXTI);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

    let mut sensor = Lis2duxs12::new_i2c(i2c, I2CAddress::I2cAddH, tim1);

    delay.delay_ms(10);

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
    while sensor.status_get().unwrap().sw_reset == 1 {}
    // Enable of Embedded Function
    sensor.init_set(Init::SensorEmbFuncOn).unwrap();

    delay.delay_ms(10);

    sensor
        .embedded_int_cfg_set(EmbeddedIntConfig::Latched)
        .unwrap();
    sensor.stpcnt_debounce_set(4).unwrap();

    let stpcnt_mode = StpcntMode {
        false_step_rej: 0,
        step_counter_enable: 1,
        step_counter_in_fifo: 0,
    };
    sensor.stpcnt_mode_set(&stpcnt_mode).unwrap();
    sensor.stpcnt_rst_step_set().unwrap();

    let int1_route = EmbPinIntRoute {
        step_det: 1,
        ..Default::default()
    };
    sensor.emb_pin_int1_route_set(&int1_route).unwrap();

    let int_config = IntConfig {
        int_cfg: IntCfg::Level,
        ..Default::default()
    };
    sensor.int_config_set(&int_config).unwrap();

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
        let status = sensor.embedded_status_get().unwrap();
        if status.is_step_det == 1 {
            let steps = sensor.stpcnt_steps_get().unwrap();
            writeln!(tx, "Steps: {}", steps).unwrap();
        }
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
