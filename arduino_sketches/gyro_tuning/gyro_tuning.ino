#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h>

Madgwick filter;
const float sample_rate = 100;
const float gyro_scale = 1; // use to increase responsiveness of filter to chanegs
                            // if too large, values will become unstable

float ax, ay, az;  // Accelerometer

float gx_offset = 0.00, gy_offset = 0.05, gz_offset = -0.24; // intended for calibration at high gyro_scale to help mitigate value drift
const int num_samples = 5000;


float gx = 0, gy = 0, gz = 0;  // Gyroscope
float measure_gx, measure_gy, measure_gz;
float pitch_g = 0.0, roll_g = 0.0, yaw_g = 0.0;

float mx, my, mz;  // Magnetometer
float mx_min =  -99.0, my_min =  -47.0, mz_min =  -46.0;
float mx_max = 27.0, my_max = 89.0, mz_max = 100.0;
bool calibrating = false;  // Set to false after collecting enough data

float base_roll = 0; // Anchor base values for tilt comparisons
float base_pitch = 0;
float base_yaw = 0;

float diff_roll = 0;
float diff_pitch = 0;
float diff_yaw = 0;

float now_time, prev_time, dt;

void calibrateGyroscope() {
  float gx_sum = 0.0, gy_sum = 0.0, gz_sum = 0.0;

  for (int i = 0; i < num_samples; i++) {
    while (!IMU.gyroscopeAvailable());

    IMU.readGyroscope(gx, gy, gz);

    gx *= gyro_scale;
    gy *= gyro_scale;
    gz *= gyro_scale;

    // Accumulate the readings
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(5); // Small delay to allow consistent sampling
  }

  // Calculate the average offset for each axis
  gx_offset = gx_sum / num_samples;
  gy_offset = gy_sum / num_samples;
  gz_offset = gz_sum / num_samples;
  Serial.print(gx_offset);
  Serial.print(" | ");
  Serial.print(gy_offset);
  Serial.print(" | ");
  Serial.println(gz_offset);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);

  IMU.begin();
  // prints out sample rate of Accel and Gyro sensors
  Serial.println("NEW SET: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  // get intitial Home tilt PRY values
  while(!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()));
  IMU.readAcceleration(ax,ay,az);
  IMU.readGyroscope(gx,gy,gz);
  IMU.readMagneticField(mx, my, mz);

  filter.begin(sample_rate);

  prev_time = millis();
  
  // RUN THIS FUNCTION HERE TO GET GYROSCOPE CALIBRATION VALUES
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
  if (yaw_mag < 0) yaw_mag += 360.0;
  if (yaw_mag > 360) yaw_mag -= 360.0;
  yaw_g = yaw_mag;

  base_roll = roll;
  base_pitch = pitch;
  base_yaw = yaw_mag;

}
// calibrating functions
void calibrating_MagSensor(){ 
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);

    if (calibrating) { // use when calibrating flag is set to true to get limit values
      // Record min and max values
      if (mx < mx_min) mx_min = mx; if (mx > mx_max) mx_max = mx;
      if (my < my_min) my_min = my; if (my > my_max) my_max = my;
      if (mz < mz_min) mz_min = mz; if (mz > mz_max) mz_max = mz;

      Serial.print("Calibrating... Min: "); Serial.print(mx_min); Serial.print(", ");
      Serial.print(my_min); Serial.print(", "); Serial.println(mz_min);
      
      Serial.print("Max: "); Serial.print(mx_max); Serial.print(", ");
      Serial.print(my_max); Serial.print(", "); Serial.println(mz_max);
    } 
    else { // test calibration results when flag is set to false
        // Compute calibration offsets and scaling factors
        float mx_offset = (mx_max + mx_min) / 2.0;
        float my_offset = (my_max + my_min) / 2.0;
        float mz_offset = (mz_max + mz_min) / 2.0;

        float mx_scale = (mx_max - mx_min) / 2.0;
        float my_scale = (my_max - my_min) / 2.0;
        float mz_scale = (mz_max - mz_min) / 2.0;

        // Apply calibration
        mx = (mx - mx_offset) / mx_scale;
        my = (my - my_offset) / my_scale;
        mz = (mz - mz_offset) / mz_scale;

        // Compute roll, pitch, yaw
        float roll  = atan2(my, mz) * 180.0 / PI;
        float pitch = atan2(-mx, sqrt(my * my + mz * mz)) * 180.0 / PI;
        float yaw   = atan2(my, mx) * 180.0 / PI;

        // Normalize yaw to 0-360 degrees
        if (yaw < 0) yaw += 360.0;

        Serial.print("Roll: "); Serial.print(roll);
        Serial.print(" Pitch: "); Serial.print(pitch);
        Serial.print(" Yaw: "); Serial.println(yaw);
    }
  }
}

// see results functions
void results_AccelOnly(){
  if(IMU.accelerationAvailable()){ // results are -4 to 4 in G's - gravity
    IMU.readAcceleration(ax, ay, az);
    float pitch = atan2(ax, sqrt( pow(ay, 2) + pow(az, 2))) * 180 / M_PI;
    float roll = atan2(ay, sqrt( pow(ax, 2) + pow(az, 2))) * 180 / M_PI;
    
    Serial.print("Accel: ");
    Serial.print(pitch);
    Serial.print(" | ");
    Serial.println(roll);
  } // gives us pretty stable roll and pitch values
}
void results_GyroOnly(){
  if(IMU.gyroscopeAvailable()){ // results are 0 to 2000 in degress/sec
    IMU.readGyroscope(gx, gy, gz);

    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    now_time = millis();
    dt = (now_time - prev_time) / 1000.0;
    prev_time = now_time;

    pitch_g += gy * dt;
    roll_g += gx * dt;
    yaw_g += gz * dt;
    Serial.print("Gyro: ");
    Serial.print(pitch_g);
    Serial.print(" | ");
    Serial.print(roll_g);
    Serial.print(" | ");
    Serial.println(yaw_g);
  } // hard to maintain home position difference of 0
}
void results_MagOnly(){
  if(IMU.magneticFieldAvailable()){ // results are 0 to 2000 in degress/sec
    IMU.readMagneticField(mx, my, mz);

    float roll = atan2(my, mz) * 180.0 / PI;
    float pitch = atan2(-mx, sqrt(my * my + mz * mz)) * 180.0 / PI;
    float yaw = atan2(my, mx) * 180.0 / PI;

    if (yaw < 0) yaw += 360.0;
    Serial.print("Mag: ");
    Serial.print(pitch);
    Serial.print(" | ");
    Serial.print(roll);
    Serial.print(" | ");
    Serial.println(yaw);
  } // results are very unstable and heavily subject to environmental conditions
}
void results_Accel_Compensating_Mag(){
  if (IMU.magneticFieldAvailable() && IMU.accelerationAvailable()) { //pitch and roll based on accelerometer, yaw baseed on magnetometer
    IMU.readMagneticField(mx, my, mz);
    IMU.readAcceleration(ax, ay, az);

    // Compute offsets and scaling factors
    float mx_offset = (mx_max + mx_min) / 2.0;
    float my_offset = (my_max + my_min) / 2.0;
    float mz_offset = (mz_max + mz_min) / 2.0;

    float mx_scale = (mx_max - mx_min) / 2.0;
    float my_scale = (my_max - my_min) / 2.0;
    float mz_scale = (mz_max - mz_min) / 2.0;

    // Apply calibration
    mx = (mx - mx_offset) / mx_scale;
    my = (my - my_offset) / my_scale;
    mz = (mz - mz_offset) / mz_scale;

    // Compute roll & pitch using accelerometer
    float roll  = atan2(ay, az) * 180.0 / PI;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Tilt-compensated yaw calculation
    float Xh = mx * cos(pitch * PI / 180.0) + mz * sin(pitch * PI / 180.0);
    float Yh = mx * sin(roll * PI / 180.0) * sin(pitch * PI / 180.0) + my * cos(roll * PI / 180.0) - mz * sin(roll * PI / 180.0) * cos(pitch * PI / 180.0);
    float yaw = atan2(Yh, Xh) * 180.0 / PI;

    // Normalize yaw to 0-360 degrees
    if (yaw < 0) yaw += 360.0;

    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" Pitch: "); Serial.print(pitch);
    Serial.print(" Yaw: "); Serial.println(yaw);
  } // better results; still not consistent enough for accurate tilt controls
}
void results_Complementary_Filter(){
  // Comp Filter pitch and roll (accel + gyro); Comp Filter Yaw (gyro + mag)
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) { 
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    now_time = millis();
    dt = (now_time - prev_time) / 1000.0;
    prev_time = now_time;

    // Gyroscope integration
    pitch_g += gy * dt;
    roll_g += gx * dt;
    yaw_g += gz * dt;

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
    float roll_a  = atan2(ay, az) * 180.0 / PI;
    float pitch_a = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    float roll = 0.02 * (roll_g) + (1 - 0.02) * roll_a;
    float pitch = 0.02 * (pitch_g) + (1 - 0.02) * pitch_a;

    // Compute tilt-compensated yaw from magnetometer
    float Xh = mx * cos(pitch * PI / 180.0) + mz * sin(pitch * PI / 180.0);
    float Yh = mx * sin(roll * PI / 180.0) * sin(pitch * PI / 180.0) + my * cos(roll * PI / 180.0) - mz * sin(roll * PI / 180.0) * cos(pitch * PI / 180.0);
    float yaw_mag = atan2(Yh, Xh) * 180.0 / PI;

    // Normalize yaw_mag to 0-360 degrees
    if (yaw_mag < 0) yaw_mag += 360.0;

    // Complementary Filter: Combine Gyro & Magnetometer Yaw
    float yaw = 0.98 * (yaw_g) + (1 - 0.98) * yaw_mag;

    // Normalize yaw to 0 - 360 degrees
    if (yaw < 0) yaw += 360.0;
    if (yaw > 360) yaw -= 360.0;

    diff_roll = roll - base_roll;
    diff_pitch = pitch - base_pitch;
    diff_yaw = yaw - base_yaw;

    // Print results
    // Serial.print("Roll: "); Serial.print(roll);
    // Serial.print(" Pitch: "); Serial.print(pitch);
    // Serial.print(" Yaw: "); Serial.println(yaw);
    
    // Orientation Change
    Serial.print("Roll: "); Serial.print(diff_roll);
    Serial.print(" Pitch: "); Serial.print(diff_pitch);
    Serial.print(" Yaw: "); Serial.println(diff_yaw);
    
  }

}
void results_Madgwick_Filter(bool axis_flag){
  // The Magdwick filter using a gradient descent algorithm to reduce error between absolute and relative predictions
  // The library is not easily tunable
  // As such a Kalman filter performed better due to its ability to be customized to our system and sensors specifically
  if(IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()){
    IMU.readAcceleration(ax,ay,az);
    IMU.readGyroscope(gx,gy,gz);
    IMU.readMagneticField(mx, my, mz);
    gx *= gyro_scale;
    gy *= gyro_scale;
    gz *= gyro_scale;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;
    if (axis_flag) filter.update(gx,gy,gz,ax,ay,az,mx,my,mz); // 9-axis magdwick filter
    else filter.updateIMU(gx,gy,gz,ax,ay,az); // use if you want only 6-axis (This actually runs better due to unreliability of mag sensor)

    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(yaw);
    Serial.print(" | ");
    Serial.print(pitch);
    Serial.print(" | ");
    Serial.println(roll);

    float diff_roll = base_roll - roll;
    float diff_pitch = base_pitch - pitch;
    float diff_yaw = base_yaw - yaw;
    // Serial.print("Orientation Change: ");
    // Serial.print(diff_yaw);
    // Serial.print(" | ");
    // Serial.print(diff_pitch);
    // Serial.print(" | ");
    // Serial.println(diff_roll);
  }


}

void loop() {
  // calibrating_MagSensor(); // use with global calibrating flag

  // OPTIONAL SEE RESULTS OF FILTERS USED PRIOR TO KALMAN FILTER
  // results_AccelOnly(); // prints predictions from accel only
  // results_GyroOnly(); // prints predictions from gyro only
  // results_MagOnly(); // prints predictions from mag only
  // results_Accel_Compensating_Mag(); // prints accel preds with mag compensated by accel prediction for yaw
  // results_Complementary_Filter(); // implements complementary filter in favour of gyro values  
  // results_Madgwick_Filter(0); // implements Madgwick filter

  delay(10);
}

