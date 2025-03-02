use image::EncodableLayout;
use serial2::SerialPort;
use std::sync::{atomic::AtomicBool, mpsc, Arc};
use std::time;
use tokio::runtime::Runtime;

use nokhwa::pixel_format::RgbFormat;
use nokhwa::{
    utils::{CameraIndex, RequestedFormat, RequestedFormatType},
    CallbackCamera, Camera,
};

enum MotorDirectionControl {
    Forward = 0,
    Backward = 1,
    Left = 2,
    Right = 3,
}

impl From<u8> for MotorDirectionControl {
    fn from(value: u8) -> Self {
        match value {
            0 => MotorDirectionControl::Forward,
            1 => MotorDirectionControl::Backward,
            2 => MotorDirectionControl::Left,
            3 => MotorDirectionControl::Right,
            _ => panic!("Invalid motor direction control value"),
        }
    }
}

mod constants;

#[derive(Debug)]
pub struct TelemetrixData {
    report_id: u8,
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
            let mut length = [0; 1];
            let result = self.serial_port.read_exact(&mut length);
            match result {
                Ok(()) => {
                    assert!(length[0] > 1);
                    let mut report_buffer = vec![0; length[0] as usize];
                    let result = self.serial_port.read_exact(&mut report_buffer);
                    match result {
                        Ok(()) => {
                            let message = TelemetrixData {
                                report_id: report_buffer[0],
                                length: report_buffer.len() as u8 - 1,
                                payload: report_buffer[1..].to_vec(),
                            };
                            self.on_message.send(message).unwrap();
                        }
                        Err(_) => {
                            println!(
                                "Error reading report data from serial port of length {}",
                                length[0]
                            );
                        }
                    }
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

struct CameraService {
    z_session: zenoh::Session,
    shutdown_flag: Arc<AtomicBool>,
}

impl CameraService {
    fn new(z_session: zenoh::Session, shutdown_flag: Arc<AtomicBool>) -> CameraService {
        CameraService {
            z_session,
            shutdown_flag,
        }
    }

    async fn run(&mut self) {
        let index = CameraIndex::Index(0);
        // request the absolute highest resolution CameraFormat that can be decoded to RGB.
        let requested =
            RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate);
        // make the camera
        let (tx, rx) = mpsc::channel();
        let mut last = std::time::Instant::now();
        let mut camera = CallbackCamera::new(index, requested, move |buf| {
            let now = std::time::Instant::now();
            let dt = now - last;
            // println!("DT = {:?}; {} fps", dt, (1.0 / dt.as_secs_f64()).round());
            last = now;
            tx.send(buf).unwrap();
        })
        .unwrap();
        camera.open_stream().unwrap();
        while !self
            .shutdown_flag
            .load(std::sync::atomic::Ordering::Relaxed)
        {
            let frame = rx.recv().unwrap();
            self.z_session
                .put("mirte/camera", frame.buffer())
                .encoding(zenoh::bytes::Encoding::IMAGE_JPEG)
                .await
                .unwrap();
        }
        self.z_session.close().await.unwrap();
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

fn enabe_distance_sensor(serial_port: &SerialPort, trigger_pin: u8, echo_pin: u8) {
    let write_buffer = [3, constants::SONAR_NEW, trigger_pin, echo_pin];
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

fn board_setup(serial_port: &SerialPort) {
    // Enable motors drivers.
    enable_pwm_pin(serial_port, 18);
    enable_pwm_pin(serial_port, 19);
    enable_pwm_pin(serial_port, 20);
    enable_pwm_pin(serial_port, 21);
    // TODO: Enable distance sensors.
    enabe_distance_sensor(serial_port, 9, 8);
    enabe_distance_sensor(serial_port, 7, 6);
}

struct MotorDriver {
    serial_port: Arc<SerialPort>,
}

impl MotorDriver {
    fn new(serial_port: Arc<SerialPort>) -> MotorDriver {
        MotorDriver { serial_port }
    }

    fn stop(&self) {
        pwm_write(&self.serial_port, 19, 0);
        pwm_write(&self.serial_port, 18, 0);
        pwm_write(&self.serial_port, 21, 0);
        pwm_write(&self.serial_port, 20, 0);
    }

    fn forward(&self, duty_cycle_percentage: u16) {
        pwm_write(&self.serial_port, 19, 0);
        pwm_write(&self.serial_port, 18, duty_cycle_percentage);
        pwm_write(&self.serial_port, 21, 0);
        pwm_write(&self.serial_port, 20, duty_cycle_percentage);
    }

    fn backward(&self, duty_cycle_percentage: u16) {
        pwm_write(&self.serial_port, 19, duty_cycle_percentage);
        pwm_write(&self.serial_port, 18, 0);
        pwm_write(&self.serial_port, 21, duty_cycle_percentage);
        pwm_write(&self.serial_port, 20, 0);
    }

    fn soft_right(&self, duty_cycle_percentage: u16) {
        pwm_write(&self.serial_port, 19, 0);
        pwm_write(&self.serial_port, 18, duty_cycle_percentage);
        pwm_write(&self.serial_port, 21, 0);
        pwm_write(&self.serial_port, 20, 0);
    }

    fn hard_right(&self, duty_cycle_percentage: u16) {
        pwm_write(&self.serial_port, 19, 0);
        pwm_write(&self.serial_port, 18, duty_cycle_percentage);
        pwm_write(&self.serial_port, 21, duty_cycle_percentage);
        pwm_write(&self.serial_port, 20, 0);
    }

    fn soft_left(&self, duty_cycle_percentage: u16) {
        pwm_write(&self.serial_port, 19, 0);
        pwm_write(&self.serial_port, 18, 0);
        pwm_write(&self.serial_port, 21, 0);
        pwm_write(&self.serial_port, 20, duty_cycle_percentage);
    }

    fn hard_left(&self, duty_cycle_percentage: u16) {
        pwm_write(&self.serial_port, 19, duty_cycle_percentage);
        pwm_write(&self.serial_port, 18, 0);
        pwm_write(&self.serial_port, 21, 0);
        pwm_write(&self.serial_port, 20, duty_cycle_percentage);
    }
}

#[tokio::main]
async fn main() -> Result<(), ()> {
    println!("Opening serial port...start");
    let (tx, rx) = mpsc::channel();
    let shutdown_flag = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown_flag))
        .map_err(|e| println!("{e}"))?;
    let serial_port =
        Arc::new(SerialPort::open("/dev/ttyUSB0", 115200).map_err(|e| println!("{e}"))?);
    let mut tmx_client =
        TelemetrixClient::new(Arc::clone(&serial_port), Arc::clone(&shutdown_flag), tx);
    println!("Opening serial port...end");

    let receiver_handler = std::thread::spawn(move || {
        tmx_client.run();
    });

    // Setup board.
    board_setup(&serial_port);

    // Zenoh session.
    let z_session = zenoh::open(zenoh::Config::default()).await.unwrap();

    let shutdown_flag_copy = Arc::clone(&shutdown_flag);
    let z_session_copy = z_session.clone();
    let camera_service_handler = std::thread::spawn(|| {
        let runtime = Runtime::new().unwrap();
        let mut camera_service = CameraService::new(z_session_copy, shutdown_flag_copy);
        runtime.block_on(async {
            camera_service.run().await;
        });
    });

    // Motor.
    let motor_driver = MotorDriver::new(Arc::clone(&serial_port));
    let teleop_subscriber = z_session
        .declare_subscriber("mirte/motor/control")
        .await
        .unwrap();
    let shutdown_flag_copy = Arc::clone(&shutdown_flag);
    let motor_control_service_handler = tokio::spawn(async move {
        while let Ok(value) = teleop_subscriber.recv() {
            let bytes = value.payload().to_bytes();
            match MotorDirectionControl::from(bytes[0]) {
                MotorDirectionControl::Forward => {
                    motor_driver.forward(50);
                    std::thread::sleep(std::time::Duration::from_millis(20));
                    motor_driver.stop();
                }
                MotorDirectionControl::Backward => {
                    motor_driver.backward(50);
                    std::thread::sleep(std::time::Duration::from_millis(20));
                    motor_driver.stop();
                }
                MotorDirectionControl::Left => {
                    motor_driver.soft_left(100);
                    std::thread::sleep(std::time::Duration::from_millis(20));
                    motor_driver.stop();
                }
                MotorDirectionControl::Right => {
                    motor_driver.soft_right(100);
                    std::thread::sleep(std::time::Duration::from_millis(20));
                    motor_driver.stop();
                }
                _ => motor_driver.stop(),
            }
            if shutdown_flag_copy.load(std::sync::atomic::Ordering::Relaxed) {
                break;
            }
        }
    });

    while !shutdown_flag.load(std::sync::atomic::Ordering::Relaxed) {
        let received_msg = rx.recv_timeout(std::time::Duration::from_secs(1));
        match received_msg {
            Ok(msg) => match msg.report_id {
                constants::SONAR_DISTANCE => {
                    let distance_cm =
                        100.0 * (msg.payload[1] as f32 + (msg.payload[2]) as f32 / 100.0);
                    println!("Distance (cm): {}", distance_cm);
                }
                _ => {
                    println!("Unknown report received");
                }
            },
            Err(_) => {
                println!("No message received from Pico!!");
                break;
            }
        }
    }

    println!("Reset board and close serial port...");

    // Reset board
    reset_board(&serial_port);

    // Shutdown receiver thread
    shutdown_flag.store(true, std::sync::atomic::Ordering::Relaxed);
    motor_control_service_handler.await.unwrap();
    receiver_handler.join().unwrap();
    camera_service_handler.join().unwrap();

    println!("Telemetrix client ended");

    Ok(())
}
