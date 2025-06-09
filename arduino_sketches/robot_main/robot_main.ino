#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>

// define pins for nRF24 module and initialize
#define CE_PIN 8
#define CSN_PIN 53
#define IRQ_PIN 2

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001"; // Address of the comm pipe
volatile bool dataAvailable = false; // flag for new data received

struct RF_Packet { // RF packet to process info received
  bool power; // Indicates whether motors should be active
  bool rotate; // Indicates whether rotation should be enabled
  float driveX; // X-direction movement
  float driveY; // Y-direction movement
  float driveW;  // Rotational movement (W angular velocity)
}__attribute__((packed));
RF_Packet receivedData = {0,0,0,0,0};

//MOTOR 1 - Front Right
const int m1_a = 30;  // H-bridge dir pins A/B
const int m1_b = 32;
const int m1_P = 5;  //pwm

//MOTOR 2 - Front Left
const int m2_a = 22;  // H-bridge dir
const int m2_b = 24;
const int m2_P = 4;  //pwm

//MOTOR 3 - Back
const int m3_a = 26;  // H-bridge dir
const int m3_b = 28;
const int m3_P = 6;  //pwm


void setup() {
  Serial.begin(9600);

  pinMode(IRQ_PIN, INPUT_PULLUP); // setting IRQ pin for nRF24

  // initializing radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);  // set power level for expected broadcast range
  radio.setChannel(100); // set RF channel to avoid common WiFi channels (0-99)
  radio.maskIRQ(1, 1, 0); // only interrupt on 'data ready' event  
  radio.startListening(); // Set module as a receiver

  // MOTOR 1
  pinMode(m1_a, OUTPUT);
  pinMode(m1_b, OUTPUT);
  pinMode(m1_P, OUTPUT);
  digitalWrite(m1_a, HIGH);
  digitalWrite(m1_b, LOW);

  // MOTOR 2
  pinMode(m2_a, OUTPUT);
  pinMode(m2_b, OUTPUT);
  pinMode(m2_P, OUTPUT);
  digitalWrite(m2_a, HIGH);
  digitalWrite(m2_b, LOW);

  // MOTOR 3
  pinMode(m3_a, OUTPUT);
  pinMode(m3_b, OUTPUT);
  pinMode(m3_P, OUTPUT);
  digitalWrite(m3_a, HIGH);
  digitalWrite(m3_b, LOW);
}

// defining global omni movement calcs
// inverse matrix to convert desired drive vector into force
float inverse_force[3][3] = {
  { -0.33, 0.58, 0.33 },
  { -0.33, -0.58, 0.33 },
  { 0.66, 0.0, 0.33 }
};
float drive_vector[3]; // stores drive commands received
float force_vector[3]; // stores calculated force needed to achieve desired drive
int dir_vector[3]; // stores dir(force) for each motor
float motor_power[3]; // stores abs(force) for each motor

float scale_factor = 2.0;  // scale motor output

// sets motor power and direction for each motor
void set_movement(float motor_power[3], int motor_dir[3]) {
  // consolidate pins for motors
  const int pwm_pins[3] = {m1_P, m2_P, m3_P};
  const int dir_pins[3][2] = {{m1_a,m1_b},{m2_a,m2_b},{m3_a,m3_b}};

  for (int i = 0; i < 3; i++) {
    if (motor_dir[i] == 1) { // set DIR outputs
      digitalWrite(dir_pins[i][0], HIGH);
      digitalWrite(dir_pins[i][1], LOW);
    } else {
      digitalWrite(dir_pins[i][0], LOW);
      digitalWrite(dir_pins[i][1], HIGH);
    }
    analogWrite(pwm_pins[i], (int)motor_power[i]); // set PWM value outputs
  }
}

void loop() {
  if (radio.available()) { // read in data when available
    radio.read(&receivedData, sizeof(receivedData));

    if (receivedData.power){ // read in drive vector values if power is on
      drive_vector[0] = receivedData.driveX;
      drive_vector[1] = receivedData.driveY;
      drive_vector[2] = receivedData.driveW;
    }
    else{ // else drive vector is zero'd out
      drive_vector[0] = 0;
      drive_vector[1] = 0;
      drive_vector[2] = 0;
    }

    if (!receivedData.rotate){ // if not rotating, zero rotation and scale up drive
      drive_vector[2] = 0;
      scale_factor = 2.0;
    }
    else{ // if rotating, scale down drive X,Y before conversion
      scale_factor = 2.0;
      drive_vector[0] *= 0.5;
      drive_vector[1] *= 0.5;
    }
    dataAvailable = false;
  }

  // Calculate Force Vectors
  for (int i = 0; i < 3; i++) {
    force_vector[i] = 0;  // Initialize result to zero
    for (int j = 0; j < 3; j++) {
      // Calculate force from matrix multiplicatoin
      force_vector[i] += inverse_force[i][j] * drive_vector[j];
    }
    if (force_vector[i] > 0) {
      dir_vector[i] = 1;
    } else {
      dir_vector[i] = -1;
    }
    force_vector[i] = abs(force_vector[i]);
  }

  // Convert to Motor PWM Output
  for (int i = 0; i < 3; i++) {
    // Normalize force_vector[i] to 0–255
    motor_power[i] = ((force_vector[i] - 0) * (255.0 - 0.0) / (1 - 0) + 0) * scale_factor;
    // Constrain the result to ensure no overflow
    motor_power[i] = constrain(motor_power[i], 0, 255);
  }
  
  set_movement(motor_power, dir_vector); // Set Motors
  // delay(10); // optional delay for debugging
}
