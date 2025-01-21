use serial2::SerialPort;
use std::sync::{atomic::AtomicBool, mpsc, Arc, Mutex};

mod constants;

pub struct TelemetrixData {
    command_id: u8,
    length: u8,
    payload: Vec<u8>,
}

pub struct TelemetrixClient {
    serial_port: Arc<SerialPort>,
    buffer: [u8; 256],
    on_message: std::sync::mpsc::Sender<TelemetrixData>,
    shutdown_flag: Arc<AtomicBool>,
}

impl TelemetrixClient {
    pub fn new(
        serial_port: Arc<SerialPort>,
        shutdown_flag: Arc<AtomicBool>,
        message_handler: std::sync::mpsc::Sender<TelemetrixData>,
    ) -> Self {
        TelemetrixClient {
            serial_port: serial_port,
            buffer: [0; 256],
            on_message: message_handler,
            shutdown_flag: shutdown_flag,
        }
    }

    pub fn run(&mut self) {
        println!("Telemetrix client started");
        let write_buffer = [1, constants::RETRIEVE_PICO_UNIQUE_ID];
        self.serial_port.write(&write_buffer).unwrap();
        let num_bytes = self.serial_port.read(&mut self.buffer).unwrap();
        let device_id = &self.buffer[2..num_bytes];
        println!("Device ID: {:?}", device_id);
        println!("Get firmware version...start");
        let write_buffer = [1, constants::GET_FIRMWARE_VERSION];
        self.serial_port.write(&write_buffer).unwrap();
        let num_bytes = self.serial_port.read(&mut self.buffer).unwrap();
        let firmware_version = &self.buffer[0..num_bytes];
        println!("Firmware version: {:?}", firmware_version);

        println!("Telemetrix client dispatcher started");
        while !self
            .shutdown_flag
            .load(std::sync::atomic::Ordering::Relaxed)
        {
            let result = self.serial_port.read(&mut self.buffer);

            match result {
                Ok(num_bytes) => {
                    assert!(num_bytes >= 2);

                    let message = TelemetrixData {
                        command_id: self.buffer[0],
                        length: self.buffer[1],
                        payload: self.buffer[2..].to_vec(),
                    };
                    self.on_message.send(message).unwrap();
                }
                Err(_) => {
                    continue;
                }
            }
        }
        println!("Telemetrix client dispatcher ended");
    }

    fn write(&self, data: &[u8]) {
        self.serial_port.write(data).unwrap();
    }
}

fn reset_board(serial_port: &SerialPort) {
    let write_buffer = [1, constants::RESET_BOARD];
    serial_port.write(&write_buffer).unwrap();
}

fn enable_pwm_pin(serial_port: &SerialPort, pin_number: u8) {
    let write_buffer = [
        3,
        constants::SET_PIN_MODE,
        pin_number,
        constants::AT_PWM_OUTPUT,
    ];
    serial_port.write(&write_buffer).unwrap();
}

fn pwm_write(serial_port: &SerialPort, pin_number: u8, duty_cycle_percentage: u16) {
    let duty_cycle: u16 =
        ((constants::MAX_RAW_DUTY_CYCLE as u32 * duty_cycle_percentage as u32) / 100 as u32) as u16;
    let value_msb = duty_cycle >> 8;
    let value_lsb = duty_cycle & 0x00FF;
    let write_buffer = [
        4,
        constants::PWM_WRITE,
        pin_number,
        value_msb as u8,
        value_lsb as u8,
    ];
    serial_port.write(&write_buffer).unwrap();
}

fn main() {
    println!("Opening serial port...start");
    let (tx, rx) = mpsc::channel();
    let shutdown_flag = Arc::new(AtomicBool::new(false));
    let serial_port = Arc::new(SerialPort::open("/dev/ttyUSB0", 115200).unwrap());
    let mut tmx_client = TelemetrixClient::new(serial_port.clone(), shutdown_flag.clone(), tx);
    println!("Opening serial port...end");

    let receiver = std::thread::spawn(move || {
        tmx_client.run();
    });

    // Trigger motors.
    enable_pwm_pin(&serial_port, 18);
    enable_pwm_pin(&serial_port, 19);
    enable_pwm_pin(&serial_port, 20);
    enable_pwm_pin(&serial_port, 21);

    // Move forward.
    pwm_write(&serial_port, 19, 0);
    pwm_write(&serial_port, 18, 50);
    pwm_write(&serial_port, 21, 0);
    pwm_write(&serial_port, 20, 50);

    std::thread::sleep(std::time::Duration::from_millis(1000));

    // Stop.
    pwm_write(&serial_port, 19, 0);
    pwm_write(&serial_port, 18, 0);
    pwm_write(&serial_port, 21, 0);
    pwm_write(&serial_port, 20, 0);

    std::thread::sleep(std::time::Duration::from_millis(500));

    // Move backward.
    pwm_write(&serial_port, 19, 50);
    pwm_write(&serial_port, 18, 0);
    pwm_write(&serial_port, 21, 50);
    pwm_write(&serial_port, 20, 0);

    std::thread::sleep(std::time::Duration::from_millis(1000));

    // Stop.
    pwm_write(&serial_port, 19, 0);
    pwm_write(&serial_port, 18, 0);
    pwm_write(&serial_port, 21, 0);
    pwm_write(&serial_port, 20, 0);

    // Reset board
    reset_board(&serial_port);

    // Shutdown receiver thread
    shutdown_flag.store(true, std::sync::atomic::Ordering::Relaxed);
    receiver.join().unwrap();

    println!("Telemetrix client ended");
}
