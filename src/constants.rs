// Commands
pub const LOOP_COMMAND: u8 = 0;
pub const SET_PIN_MODE: u8 = 1;
pub const DIGITAL_WRITE: u8 = 2;
pub const PWM_WRITE: u8 = 3;
pub const MODIFY_REPORTING: u8 = 4;
pub const GET_FIRMWARE_VERSION: u8 = 5;
pub const RETRIEVE_PICO_UNIQUE_ID: u8 = 6;
pub const SERVO_ATTACH: u8 = 7;
pub const SERVO_WRITE: u8 = 8;
pub const SERVO_DETACH: u8 = 9;
pub const I2C_BEGIN: u8 = 10;
pub const I2C_READ: u8 = 11;
pub const I2C_WRITE: u8 = 12;
pub const SONAR_NEW: u8 = 13;
pub const DHT_NEW: u8 = 14;
pub const STOP_ALL_REPORTS: u8 = 15;
pub const ENABLE_ALL_REPORTS: u8 = 16;
pub const RESET_DATA: u8 = 17;
pub const RESET_BOARD: u8 = 18;
pub const INITIALIZE_NEO_PIXELS: u8 = 19;
pub const SHOW_NEO_PIXELS: u8 = 20;
pub const SET_NEO_PIXEL: u8 = 21;
pub const CLEAR_ALL_NEO_PIXELS: u8 = 22;
pub const FILL_ALL_NEO_PIXELS: u8 = 23;
pub const SPI_INIT: u8 = 24;
pub const SPI_WRITE_BLOCKING: u8 = 25;
pub const SPI_READ_BLOCKING: u8 = 26;
pub const SPI_SET_FORMAT: u8 = 27;
pub const SPI_CS_CONTROL: u8 = 28;
pub const SET_SCAN_DELAY: u8 = 29;
pub const ENCODER_NEW: u8 = 30;
pub const SENSOR_NEW: u8 = 31;
pub const PING: u8 = 32;
pub const MODULE_NEW: u8 = 33;
pub const MODULE_DATA: u8 = 34;

// Reports
pub const DIGITAL_REPORT: u8 = DIGITAL_WRITE;
pub const ANALOG_REPORT: u8 = 3;
pub const FIRMWARE_REPORT: u8 = GET_FIRMWARE_VERSION;
pub const UNIQUE_ID_REPORT: u8 = RETRIEVE_PICO_UNIQUE_ID;
pub const SERVO_UNAVAILABLE: u8 = SERVO_ATTACH;
pub const I2C_WRITE_FAILED: u8 = 8;
pub const I2C_READ_FAILED: u8 = 9;
pub const I2C_READ_REPORT: u8 = 10;
pub const SONAR_DISTANCE: u8 = 11;
pub const DHT_REPORT: u8 = 12;
pub const SPI_REPORT: u8 = 13;
pub const ENCODER_REPORT: u8 = 14;
pub const DEBUG_PRINT: u8 = 99;
pub const SENSOR_REPORT: u8 = 20;
pub const PONG_REPORT: u8 = PING;
pub const MODULE_REPORT: u8 = MODULE_DATA;
pub const TELEMETRIX_VERSION: &str = "1.5";

// Reporting control
pub const REPORTING_DISABLE_ALL: u8 = 0;
pub const REPORTING_ANALOG_ENABLE: u8 = 1;
pub const REPORTING_DIGITAL_ENABLE: u8 = 2;
pub const REPORTING_ANALOG_DISABLE: u8 = 3;
pub const REPORTING_DIGITAL_DISABLE: u8 = 4;

// Pin mode definitions
pub const AT_INPUT: u8 = 0;
pub const AT_OUTPUT: u8 = 1;
pub const AT_PWM_OUTPUT: u8 = 2;
pub const AT_INPUT_PULLUP: u8 = 3;
pub const AT_INPUT_PULL_DOWN: u8 = 4;
pub const AT_ANALOG: u8 = 5;
pub const AT_SERVO: u8 = 6;
pub const AT_SONAR: u8 = 7;
pub const AT_DHT: u8 = 8;
pub const AT_I2C: u8 = 9;
pub const AT_NEO_PIXEL: u8 = 10;
pub const AT_SPI: u8 = 11;
pub const AT_ENCODER: u8 = 12;
pub const AT_MODE_NOT_SET: u8 = 255;

// Other constants
pub const I2C_NO_REGISTER: u8 = 254;
pub const NUMBER_OF_DIGITAL_PINS: u8 = 100;
pub const MAX_PWM_PINS_ACTIVE: u8 = 16;
pub const NUMBER_OF_ANALOG_PINS: u8 = 20;
pub const MAX_RAW_DUTY_CYCLE: u16 = 20000;
pub const MIN_SERVO_DUTY_CYCLE: u8 = 0;
pub const MAX_SERVO_DUTY_CYCLE: u8 = 1;
pub const MAX_SONARS: u8 = 4;
pub const MAX_DHTS: u8 = 2;
pub const MAX_ENCODERS: u8 = 4;

// DHT Report sub-types
pub const DHT_DATA: u8 = 0;
pub const DHT_ERROR: u8 = 1;

// NeoPixel color positions
pub const RED: u8 = 0;
pub const GREEN: u8 = 1;
pub const BLUE: u8 = 2;

// Convert Python enums to Rust enums
#[derive(Debug, Clone, Copy)]
pub enum SensorTypes {
    GPS,
    LoadCell,
    MPU9250,
    TofVL53,
    VEML6040,  // Color sensor
    ADXL345,   // 3 axis accel
    INA226,
}

#[derive(Debug, Clone, Copy)]
pub enum ModuleTypes {
    PCA9685,
    HiwonderServo,
    ShutdownRelay,
}