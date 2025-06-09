#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

// define pins for nRF24 modul
#define CE_PIN 2
#define CSN_PIN 3

// define pins for button inputs
#define power_button 5
#define rotate_button 4

// Define RGB LED pins (Common Anode)
const int greenPin = A0;  // green = brown on A0
const int redPin = A1;    // red = blue on A1
const int bluePin = A2;   // blue = green on A2
const int powerPin = D9;  // power to hold 3.3V HIGH value

// *ENUM MUST BE AT THE TOP OF THE FILE BEFORE STRUCTS / FUNCS
enum ledColour { // enumerated list of all RGB colours
  Red,
  Green,
  Blue,
  Yellow,
  Cyan,
  Magenta,
  White,
  Off
};
ledColour curColour = Magenta; // default colour state begins with Magenta on startup

// intialize radio for nRF24 module
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";  // Address of the pipe

struct RF_Packet { // RF packet to process info received
  bool power; // Indicates whether motors should be active
  bool rotate; // Indicates whether rotation should be enabled
  float driveX; // X-direction movement
  float driveY; // Y-direction movement
  float driveW; // Rotational movement (W angular velocity)
}__attribute__((packed));
RF_Packet dataToSend = {true, true, 1.0, 1.5, 2.0}; // basic default values for testing

// flags that are updated on button presses
volatile bool calibrate_flag = false;
volatile bool power_buttonPressed = false;
volatile bool rotate_buttonPressed = false;
static unsigned long lastInterruptTime = 0; // used to ensure a single tap does not cause multiple interrupt calls

// Interrupt Service Routines for Buttons
void handlePowerPress(){
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 100) {
    power_buttonPressed = !power_buttonPressed; // change state
    if(power_buttonPressed) calibrate_flag = true; // recalibrate Home Position of controller when power button is pressed on
    lastInterruptTime = interruptTime;
  }
}

void handleRotatePress() {
  unsigned long interruptTime = millis();
  
  // additional 0.3s delay requirement here due to sticky rotate button hardware
  if (interruptTime - lastInterruptTime > 300) { 
    rotate_buttonPressed = !rotate_buttonPressed;
    if (rotate_buttonPressed) calibrate_flag = true; // recalibrate Home Position of controller when rotate button is pressed on
  
    lastInterruptTime = interruptTime;
  }
}

// TILT MOTION CONTROL SETUP

float ax, ay, az;  // Accelerometer xyz values; Does not need calibration

float gx = 0, gy = 0, gz = 0;  // Gyro xyz values
float pitch_g = 0.0, roll_g = 0.0, yaw_g = 0.0; // global Gyro PRY values
float comp_yaw_g = 0.0; // additional complementary filter yaw value
float gx_offset = 0.00, gy_offset = 0.05, gz_offset = -0.24; // intended for calibration at high gyro_scale to help mitigate value drift
const int num_samples = 5000; // sample count used during Gryo calibration

float mx, my, mz;  // Magnetometer xyz values
// Magnetometer calibration values to help mitigate value drift
float mx_min =  -99.0, my_min =  -47.0, mz_min =  -46.0;
float mx_max = 27.0, my_max = 89.0, mz_max = 100.0;
bool calibrating = false;  // used during magnetometer calibration (Set to false after collecting enough data)

// store Home and Difference PRY values for tilt direction and magnitude
float base_roll = 0; 
float base_pitch = 0;
float base_yaw = 0;
float diff_roll = 0;
float diff_pitch = 0;
float diff_yaw = 0;

float now_time, prev_time, dt; // used to calculate dt on a scale of ms

// Kalman filter class setup
class Kalman {
public:
  Kalman(float Q_angle_in, float Q_bias_in, float R_measure_in) {
    Q_angle = Q_angle_in; // Process noise covariance (responsiveness)
    Q_bias = Q_bias_in;  // Bias covariance (adaption)
    R_measure = R_measure_in; // Measurement noise covariance (how much bias towards absolute estimator vs predictor)
    
    angle = 0.0;
    bias = 0.0;
    rate = 0.0;
    P[0][0] = 1.0; P[0][1] = 0.0; P[1][0] = 0.0; P[1][1] = 1.0;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    // Predict
    rate = newRate - bias;
    angle += dt * rate;

    // Update covariance matrix
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Compute Kalman gain to model noise of system
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Update estimate to a more accurate prediction
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // Update covariance matrix
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

private:
  float Q_angle, Q_bias, R_measure;
  float angle, bias, rate;
  float P[2][2];
};
Kalman kalmanPitch(0.01,0.003,0.02); // to be used with Accel and Gyro
Kalman kalmanRoll(0.01,0.003,0.02); // to be used with Accel and Gyro
Kalman kalmanYaw(0.01,0.003,0.001); // to be used wtih Mag and Gryo

// Load Pulser Gate Logic setup
const int pulse_gate = D6; // pin to control gate
bool gateState = LOW; // state of wheter the gate is open or closed
const int pulse_ON = 1000; // pulse battery with increased load for 1 second
const int pulse_OFF = 30000; // wait between pulses for 30 seconds
unsigned long interval = pulse_OFF; // time measurement variables
unsigned long pulse_prevTime = 0;

// sets RGB LED colour using PWM signals to control RGB value pins
// could be more advanced but only uses main enumerated colours for LED strength 
void set_ledColour(ledColour set_colour) {
  switch (set_colour) {
    case Red:
      analogWrite(redPin, 0);
      analogWrite(greenPin, 255);
      analogWrite(bluePin, 255);
      break;
    case Green:
      analogWrite(redPin, 255);
      analogWrite(greenPin, 0);
      analogWrite(bluePin, 255);
      break;
    case Blue:
      analogWrite(redPin, 255);
      analogWrite(greenPin, 255);
      analogWrite(bluePin, 0);
      break;
    case Yellow:
      analogWrite(redPin, 0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin, 255);
      break;
    case Cyan:
      analogWrite(redPin, 255);
      analogWrite(greenPin, 0);
      analogWrite(bluePin, 0);
      break;
    case Magenta:
      analogWrite(redPin, 0);
      analogWrite(greenPin, 255);
      analogWrite(bluePin, 0);
      break;
    case White:
      analogWrite(redPin, 0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin, 0);
      break;
    case Off:
      analogWrite(redPin, 255);
      analogWrite(greenPin, 255);
      analogWrite(bluePin, 255);
      break;
  }
}

// BLE setup
// findable advertising service and characteristic ID for data
const char *deviceServiceUUID = "891763a1-0d7f-48f1-b9fc-e403014730a1";
const char *deviceServicePacketCharUUID = "891763a3-0d7f-48f1-b9fc-e403014730a2";
BLEService dataService(deviceServiceUUID);
BLEStringCharacteristic dataPacketChar(deviceServicePacketCharUUID, BLERead | BLENotify, sizeof(dataToSend) + 5);

String structToString(RF_Packet &data){ // converts RF packet struct into a deconstructable string format for BLE
  String result = String(data.power) + "/";
  result += String(data.rotate) + "/";
  result += String(data.driveX) + "/";
  result += String(data.driveY) + "/";
  result += String(data.driveW) + "/";
  return result;
}

void setup() {
  Serial.begin(9600);

  // set device and local name for BLE advertising
  BLE.setDeviceName("KiwiGlove");
  BLE.setLocalName("KiwiGlove");
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setAdvertisedService(dataService);
  dataService.addCharacteristic(dataPacketChar);
  BLE.addService(dataService);
  String dataBytes = structToString(dataToSend);
  dataPacketChar.setValue(dataBytes);
  BLE.advertise();

  // set LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin,HIGH);

  //set Load Pulser Gate pins
  pinMode(pulse_gate,OUTPUT);
  digitalWrite(pulse_gate,gateState);
  
  // set Button Interrupt pins
  pinMode(power_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(power_button), handlePowerPress, FALLING); // only when pin drops to gnd
  pinMode(rotate_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rotate_button), handleRotatePress, FALLING);

  // intializing radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH); // set power level for expected broadcast range
  radio.setChannel(100); // set RF channel to avoid common WiFi channels (0-99)
  radio.stopListening();  // Set module as transmitter

  // initialize IMU (Accel, Gyro and Mag)
  IMU.begin();

  while(!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable())); // wait until sensors are ready
  // read in and initialize Home position starting values
  IMU.readAcceleration(ax,ay,az);
  IMU.readGyroscope(gx,gy,gz);
  IMU.readMagneticField(mx, my, mz);

  prev_time = millis();
  // calibrateGyroscope();

  // Magnetometer Calibration
  float mx_offset = (mx_max + mx_min) / 2.0;
  float my_offset = (my_max + my_min) / 2.0;
  float mz_offset = (mz_max + mz_min) / 2.0;

  float mx_scale = (mx_max - mx_min) / 2.0;
  float my_scale = (my_max - my_min) / 2.0;
  float mz_scale = (mz_max - mz_min) / 2.0;

  mx = (mx - mx_offset) / mx_scale;
  my = (my - my_offset) / my_scale;
  mz = (mz - mz_offset) / mz_scale;

  // Compute roll & pitch from accelerometer
  float roll  = atan2(ay, az) * 180.0 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Compute tilt-compensated yaw from magnetometer
  float Xh = mx * cos(pitch * PI / 180.0) + mz * sin(pitch * PI / 180.0);
  float Yh = mx * sin(roll * PI / 180.0) * sin(pitch * PI / 180.0) + my * cos(roll * PI / 180.0) - mz * sin(roll * PI / 180.0) * cos(pitch * PI / 180.0);
  float yaw_mag = atan2(Yh, Xh) * 180.0 / PI;

  // Normalize yaw_mag to 0-360 degrees
  // if (yaw_mag < 0) yaw_mag += 360.0;
  // if (yaw_mag > 360) yaw_mag -= 360.0;
  roll_g = roll;
  pitch_g = pitch;
  yaw_g = yaw_mag;
  comp_yaw_g = yaw_mag;

  base_roll = roll;
  base_pitch = pitch;
  base_yaw = yaw_mag;
}

void loop() {
  
  // set LED based on state of Buttons
  if (power_buttonPressed) curColour = rotate_buttonPressed ? Yellow : Green;
  else curColour = Red;
  set_ledColour(curColour);

  // Load Pulser circuit check (swaps between 1 and 30 seconds)
  unsigned long pulse_curTime = millis();
  if (pulse_curTime - pulse_prevTime >= interval){
    pulse_prevTime = pulse_curTime;
    gateState = gateState ? LOW : HIGH;
    digitalWrite(pulse_gate,gateState);
    interval = gateState ? pulse_ON : pulse_OFF;
  }

  // if calibrate flag is set by buttons, then we update Home tilt position
  if (calibrate_flag){
    // Magnetometer Calibration
    float mx_offset = (mx_max + mx_min) / 2.0;
    float my_offset = (my_max + my_min) / 2.0;
    float mz_offset = (mz_max + mz_min) / 2.0;

    float mx_scale = (mx_max - mx_min) / 2.0;
    float my_scale = (my_max - my_min) / 2.0;
    float mz_scale = (mz_max - mz_min) / 2.0;

    mx = (mx - mx_offset) / mx_scale;
    my = (my - my_offset) / my_scale;
    mz = (mz - mz_offset) / mz_scale;

    // Compute roll & pitch from accelerometer
    float roll  = atan2(ay, az) * 180.0 / PI;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Compute tilt-compensated yaw from magnetometer
    float Xh = mx * cos(pitch * PI / 180.0) + mz * sin(pitch * PI / 180.0);
    float Yh = mx * sin(roll * PI / 180.0) * sin(pitch * PI / 180.0) + my * cos(roll * PI / 180.0) - mz * sin(roll * PI / 180.0) * cos(pitch * PI / 180.0);
    float yaw_mag = atan2(Yh, Xh) * 180.0 / PI;

    // Normalize yaw_mag to 0-360 degrees
    // if (yaw_mag < 0) yaw_mag += 360.0;
    // if (yaw_mag > 360) yaw_mag -= 360.0;
    roll_g = roll;
    pitch_g = pitch;
    yaw_g = yaw_mag;
    comp_yaw_g = yaw_mag;

    base_roll = roll;
    base_pitch = pitch;
    base_yaw = yaw_mag;

    calibrate_flag = false;
  }

  // when all sensors are available, calculate current PRY values and get difference with Home PRY values
  // use difference to calcuate desired drive vector values
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // apply Gyro offsets
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    // calculate dt
    now_time = millis();
    dt = (now_time - prev_time) / 1000.0;
    prev_time = now_time;

    // calculate independent Gyro Yaw Prediction to be used with Kalman_Mag prediction later
    comp_yaw_g += gz * dt;

    // Apply Magnetometer Calibration
    float mx_offset = (mx_max + mx_min) / 2.0;
    float my_offset = (my_max + my_min) / 2.0;
    float mz_offset = (mz_max + mz_min) / 2.0;

    float mx_scale = (mx_max - mx_min) / 2.0;
    float my_scale = (my_max - my_min) / 2.0;
    float mz_scale = (mz_max - mz_min) / 2.0;

    mx = (mx - mx_offset) / mx_scale;
    my = (my - my_offset) / my_scale;
    mz = (mz - mz_offset) / mz_scale;

    // Compute roll & pitch from accelerometer
    float roll_a  = atan2(ay, az) * 180.0 / PI;
    float pitch_a = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    
    // Compute final Roll and Pitch with Accel predictions and Gyro values in Kalman Filter
    float pitch = kalmanPitch.getAngle(pitch_a, gx, dt);
    float roll = kalmanRoll.getAngle(roll_a, gy, dt);

    // Compute tilt-compensated yaw from magnetometer
    float Xh = mx * cos(pitch * PI / 180.0) + mz * sin(pitch * PI / 180.0);
    float Yh = mx * sin(roll * PI / 180.0) * sin(pitch * PI / 180.0) + my * cos(roll * PI / 180.0) - mz * sin(roll * PI / 180.0) * cos(pitch * PI / 180.0);
    float yaw_mag = atan2(Yh, Xh) * 180.0 / PI;

    // Compute Kalman yaw with Mag prediction and Gyro values in Kalman Filter
    float kalman_yaw = kalmanYaw.getAngle(yaw_mag, gz, dt);
    // Combine Kalman yaw with Gyro yaw prediction (much more reliable) to get final yaw
    float yaw = 0.98 * (comp_yaw_g) + (1 - 0.98) * kalman_yaw;
    
    // Compute difference between Home and Current PRY values
    diff_roll = roll - base_roll;
    diff_pitch = pitch - base_pitch;
    diff_yaw = yaw - base_yaw;

    // Update drive vector in RF packet based on PRY values between -45 and 45 degrees of range mapped to -1 to 1 in the user's frame of reference
    dataToSend.driveW = ((constrain(abs(diff_yaw),0,45) - 0.0) * (1.0 - 0) / (45.0 - 0) + 0) * (diff_yaw > 0 ? -1 : 1);
    dataToSend.driveX = ((constrain(abs(diff_pitch),0,45) - 0.0) * (1.0 - 0) / (45.0 - 0) + 0) * (diff_pitch > 0 ? -1 : 1);
    dataToSend.driveY = ((constrain(abs(diff_roll),0,45) - 0.0) * (1.0 - 0) / (45.0 - 0) + 0) * (diff_roll > 0 ? 1 : -1);     
  }
  // Update button states in RF packet
  dataToSend.power = power_buttonPressed;
  dataToSend.rotate = rotate_buttonPressed;
  
  bool success = radio.write(&dataToSend, sizeof(dataToSend)); // send data over RF to Robot

  // check for BLE connections to arduino device
  BLEDevice central = BLE.central();
  BLE.poll();
  if(central){ // if connection conver RF packet to a string and update advertised information on BLE service
    String dataBytes = structToString(dataToSend);
    dataPacketChar.writeValue(dataBytes); // BLE service will automatically notify connected device of an update (flag set: BLENotify)
  }

  delay(10); // artificially introduced delay, otherwise Robot nRF24 module serial buffer would get overflow  
}


