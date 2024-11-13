// The BNO080 supports six communication channels:
pub const NUM_CHANNELS: usize = 6;
pub const CHANNEL_COMMAND: u8 = 0;
pub const CHANNEL_EXECUTABLE: u8 = 1;
pub const CHANNEL_CONTROL: u8 = 2;
pub const CHANNEL_REPORTS: u8 = 3;
pub const CHANNEL_WAKE_REPORTS: u8 = 4;
pub const CHANNEL_GYRO: u8 = 5;

// All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
// These are used for low level communication with the sensor, on channel 2
pub const SHTP_REPORT_FORCE_SENSOR_FLUSH: u8 = 0xF0;
pub const SHTP_COMMAND_RESPONSE: u8 = 0xF1;
pub const SHTP_COMMAND_REQUEST: u8 = 0xF2;
pub const SHTP_REPORT_FRS_READ_RESPONSE: u8 = 0xF3;
pub const SHTP_REPORT_FRS_READ_REQUEST: u8 = 0xF4;
pub const SHTP_REPORT_PRODUCT_ID_RESPONSE: u8 = 0xF8;
pub const SHTP_REPORT_PRODUCT_ID_REQUEST: u8 = 0xF9;
pub const SHTP_REPORT_BASE_TIMESTAMP: u8 = 0xFB;
pub const SHTP_REPORT_GET_FEATURE_RESPONSE: u8 = 0xFC;
pub const SHTP_REPORT_SET_FEATURE_COMMAND: u8 = 0xFD;
pub const SHTP_REPORT_GET_FEATURE_REQUEST: u8 = 0xFE;

// All the different sensors and features we can get reports from
// These are used when enabling a given sensor
pub const SENSOR_REPORTID_ACCELEROMETER: u8 = 0x01;
pub const SENSOR_REPORTID_GYROSCOPE: u8 = 0x02;
pub const SENSOR_REPORTID_MAGNETIC_FIELD: u8 = 0x03;
pub const SENSOR_REPORTID_LINEAR_ACCELERATION: u8 = 0x04;
pub const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;
pub const SENSOR_REPORTID_GRAVITY: u8 = 0x06;
pub const SENSOR_REPORTID_UNCALIBRATED_GYRO: u8 = 0x07;
pub const SENSOR_REPORTID_GAME_ROTATION_VECTOR: u8 = 0x08;
pub const SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR: u8 = 0x09;
pub const SENSOR_REPORTID_PRESSURE: u8 = 0x0A; // hectopascals
pub const SENSOR_REPORTID_AMBIENT_LIGHT: u8 = 0x0B; // lux
pub const SENSOR_REPORTID_HUMIDITY: u8 = 0x0C; // percent
pub const SENSOR_REPORTID_PROXIMITY: u8 = 0x0D; // centimeters
pub const SENSOR_REPORTID_TEMPERATURE: u8 = 0x0E; // degrees C
pub const SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR: u8 = 0x2A;
pub const SENSOR_REPORTID_TAP_DETECTOR: u8 = 0x10;
pub const SENSOR_REPORTID_STEP_COUNTER: u8 = 0x11;
pub const SENSOR_REPORTID_STABILITY_CLASSIFIER: u8 = 0x13;
pub const SENSOR_REPORTID_RAW_ACCELEROMETER: u8 = 0x14;
pub const SENSOR_REPORTID_RAW_GYROSCOPE: u8 = 0x15;
pub const SENSOR_REPORTID_RAW_MAGNETOMETER: u8 = 0x16;
pub const SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER: u8 = 0x1E;
pub const SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR: u8 = 0x28;
pub const SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR: u8 = 0x29;

// Record IDs from figure 29, page 29 reference manual
// These are used to read the metadata for each sensor type
pub const FRS_RECORDID_ACCELEROMETER: u16 = 0xE302;
pub const FRS_RECORDID_GYROSCOPE_CALIBRATED: u16 = 0xE306;
pub const FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED: u16 = 0xE309;
pub const FRS_RECORDID_ROTATION_VECTOR: u16 = 0xE30B;

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
pub const EXECUTABLE_RESET_COMPLETE: u8 = 0x1;

// Command IDs from section 6.4, page 42
// These are used to calibrate, initialize, set orientation, tare etc the sensor
pub const COMMAND_ERRORS: u8 = 1;
pub const COMMAND_COUNTER: u8 = 2;
pub const COMMAND_TARE: u8 = 3;
pub const COMMAND_INITIALIZE: u8 = 4;
pub const COMMAND_DCD: u8 = 6;
pub const COMMAND_ME_CALIBRATE: u8 = 7;
pub const COMMAND_DCD_PERIOD_SAVE: u8 = 9;
pub const COMMAND_OSCILLATOR: u8 = 10;
pub const COMMAND_CLEAR_DCD: u8 = 11;

pub const CALIBRATE_ACCEL: u8 = 0;
pub const CALIBRATE_GYRO: u8 = 1;
pub const CALIBRATE_MAG: u8 = 2;
pub const CALIBRATE_PLANAR_ACCEL: u8 = 3;
pub const CALIBRATE_ACCEL_GYRO_MAG: u8 = 4;
pub const CALIBRATE_STOP: u8 = 5;

pub const TARE_NOW: u8 = 0;
pub const TARE_PERSIST: u8 = 1;
pub const TARE_SET_REORIENTATION: u8 = 2;

pub const TARE_AXIS_ALL: u8 = 0x07;
pub const TARE_AXIS_Z: u8 = 0x04;

pub const TARE_ROTATION_VECTOR: u8 = 0;
pub const TARE_GAME_ROTATION_VECTOR: u8 = 1;
pub const TARE_GEOMAGNETIC_ROTATION_VECTOR: u8 = 2;
pub const TARE_GYRO_INTEGRATED_ROTATION_VECTOR: u8 = 3;
pub const TARE_AR_VR_STABILIZED_ROTATION_VECTOR: u8 = 4;
pub const TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR: u8 = 5;

// Packets can be up to 32k, but we don't have that much RAM.
pub const MAX_PACKET_SIZE: u8 = 128;
// This is in words. There can be many, but we mostly only care about the first 9 (Qs, range, etc)
pub const MAX_METADATA_SIZE: u8 = 9;

// These Q values are defined in the datasheet but can also be obtained by querying the meta data records
pub const ROTATION_VECTOR_Q1: u8 = 14;
pub const ROTATION_VECTOR_ACCURACY_Q1: u8 = 12; //Heading accuracy estimate in radians. The Q point is 12.
pub const ACCELEROMETER_Q1: u8 = 8;
pub const LINEAR_ACCELEROMETER_Q1: u8 = 8;
pub const GYRO_Q1: u8 = 9;
pub const MAGNETOMETER_Q1: u8 = 4;
pub const ANGULAR_VELOCITY_Q1: u8 = 10;
pub const GRAVITY_Q1: u8 = 8;
pub const PRESSURE_Q1: u8 = 20;
pub const AMBIENT_LIGHT_Q1: u8 = 8;
pub const HUMIDITY_Q1: u8 = 8;
pub const PROXIMITY_Q1: u8 = 4;
pub const TEMPERATURE_Q1: u8 = 7;

// executable/device channel responses
// Figure 1-27: SHTP executable commands and response
pub const EXECUTABLE_DEVICE_CMD_UNKNOWN: u8 = 0;
pub const EXECUTABLE_DEVICE_CMD_RESET: u8 = 1;
pub const EXECUTABLE_DEVICE_CMD_ON: u8 = 2;
pub const EXECUTABLE_DEVICE_CMD_SLEEP: u8 = 3;
