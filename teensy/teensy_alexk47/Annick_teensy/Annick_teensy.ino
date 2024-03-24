#include <Encoder.h>
#include <cmath>
#include "SPISlave_T4.h"

SPISlave_T4 mySPI(0, SPI_32_BITS);

uint32_t data = 13;

Encoder encl(25, 26);
Encoder encr(30, 31);

int LEVEL_SHIFTER = 2;
const uint8_t C_l = 14;
const uint8_t D_l = 15;
const uint8_t PWM_l = 22;
const uint8_t CURRENT_l = 40;
const uint8_t C_r = 3;
const uint8_t D_r = 4;
const uint8_t PWM_r = 23;
const uint8_t CURRENT_r = 41;

float mass = 7.0;
int cpr = 2048 * 4;
float wheel_radius = 0.03;
float max_voltage = 24.0;
float max_current = 20.0;
float k_phi = 3.962 * 1e-3 * 60.0 / (2.0 * PI);
float R_a = 5.84;
float K_v = 1.0 / 39000.0 * 60.0 / (2.0 * PI);
float J_r = mass / 2.0 * wheel_radius * wheel_radius / (19.0 * 19.0) + 1.2e-6;
float tau_m = J_r / K_v;

float controller_time_constant = 0.005;

float Ki = (R_a * K_v) / (k_phi * controller_time_constant);
float Kp = tau_m * Ki;

int previous_time;
int previous_encoder_left;
int previous_encoder_right;

float left_integral_term = 0.0;
float right_integral_term = 0.0;
float left_reference_speed = 0.0;
float right_reference_speed = 0.0;
float time_interval;
float max_recorded_speed = 0.0;

float left_measured_speed_buffer[] = {0.0, 0.0, 0.0};
float left_filtered_speed_buffer[] = {0.0, 0.0};

float right_measured_speed_buffer[] = {0.0, 0.0, 0.0};
float right_filtered_speed_buffer[] = {0.0, 0.0};

void setup() {
  Serial.begin(19200);
  mySPI.begin(MSBFIRST, SPI_MODE2);
  mySPI.onReceive(receiveEvent);

  analogWriteFrequency(PWM_l, 40e3);
  analogWriteFrequency(PWM_r, 40e3);

  pinMode(C_l, OUTPUT);
  pinMode(D_l, OUTPUT);
  pinMode(PWM_l, OUTPUT);

  pinMode(C_r, OUTPUT);
  pinMode(D_r, OUTPUT);
  pinMode(PWM_r, OUTPUT);

  pinMode(CURRENT_l, INPUT);
  pinMode(CURRENT_r, INPUT);

  pinMode(LEVEL_SHIFTER, OUTPUT);
  digitalWrite(LEVEL_SHIFTER, HIGH);

  previous_time = micros();
  previous_encoder_left = encl.read();
  previous_encoder_right = encr.read();
}

void loop() {
  int current_time = micros();
  if (current_time - previous_time > 100) {
    time_interval = (current_time - previous_time) * 1e-6;

    int left_encoder = encl.read();
    float left_measured_speed = static_cast<float>(left_encoder - previous_encoder_left) / cpr * 2.0 * PI / time_interval;
    float left_filtered_speed = left_filter(left_measured_speed);

    int right_encoder = encr.read();
    float right_measured_speed = static_cast<float>(right_encoder - previous_encoder_right) / cpr * 2.0 * PI / time_interval;
    float right_filtered_speed = right_filter(right_measured_speed);

    float left_voltage = PI_motor_controller(left_filtered_speed, 19.0*left_reference_speed, &left_integral_term, time_interval);
    float right_voltage = PI_motor_controller(right_filtered_speed, 19.0*right_reference_speed, &right_integral_term, time_interval);

    // float test_ref = 1.0*(6.0 + 2.0*sin(static_cast<float>(current_time)*1e-6*3.1415));

    // float left_voltage = PI_motor_controller(left_filtered_speed, 19.0*test_ref, &left_integral_term, time_interval);
    // float right_voltage = PI_motor_controller(right_filtered_speed, 19.0*test_ref, &right_integral_term, time_interval);

    // sets each voltage to zero if it is less than 0.1
    if (fabs(left_voltage) < 0.1) {
      left_voltage = 0.0;
    }
    if (fabs(right_voltage) < 0.1) {
      right_voltage = 0.0;
    }

    float left_PWM = min(255, left_voltage / max_voltage * 255);
    float right_PWM = min(255, right_voltage / max_voltage * 255);

    int left_direction = (left_PWM > 0) ? -1 : 1;
    int right_direction = (right_PWM > 0) ? -1 : 1;

    setMotor(left_direction, fabs(left_PWM), PWM_l, C_l, D_l);
    setMotor(right_direction, fabs(right_PWM), PWM_r, C_r, D_r);

    previous_time = current_time;
    previous_encoder_left = left_encoder;
    previous_encoder_right = right_encoder;

    if(max_recorded_speed<left_measured_speed){
      max_recorded_speed = left_measured_speed;
    }
    if(max_recorded_speed<right_measured_speed){
      max_recorded_speed = right_measured_speed;
    }

    Serial.print("Left :");
    Serial.print(left_reference_speed);
    Serial.print(" Right:");
    Serial.print(right_reference_speed);
    Serial.print("   Left measured:");
    Serial.print(left_measured_speed);
    Serial.print(" Right measured:");
    Serial.print(right_measured_speed);
    Serial.print("   Max speed:");
    Serial.print(max_recorded_speed);
    Serial.print("   Kp:");
    Serial.print(Kp);
    Serial.print("   Ki:");
    Serial.println(Ki);
  }
}

float PI_motor_controller(float omega_mes, float omega_ref, float *integral_term, float time_interval) {
  float error = omega_ref - omega_mes;

  *integral_term += error * time_interval;

  if (*integral_term > max_voltage / Ki) {
    *integral_term = max_voltage / Ki;
  } else if (*integral_term < -max_voltage / Ki) {
    *integral_term = -max_voltage / Ki;
  }

  float output_voltage = Kp * error + Ki * (*integral_term) + k_phi * omega_mes;

  if ((output_voltage - k_phi * omega_mes) / R_a > max_current) {
    output_voltage = max_current * R_a + k_phi * omega_mes;
  } else if ((output_voltage - k_phi * omega_mes) / R_a < -max_current) {
    output_voltage = -max_current * R_a + k_phi * omega_mes;
  }

  return output_voltage;
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);

  if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


void receiveEvent() {
  // Print message between [  ]
  Serial.print("[");
  //When there is data to read
  while ( mySPI.available() ) {
    // Get data
    data = mySPI.popr();
    // push it to send buffer
    mySPI.pushr(data);

    bytesToInts(data, &right_reference_speed, &left_reference_speed);

    // Print data
    Serial.print(data, HEX);
  }
  Serial.println("]");
}

void bytesToInts(uint32_t data, float *value1, float *value2) {
  uint8_t bytes[4];
  uint32_to_uint8_array(data, bytes);

  *value1 = ((((bytes[0] << 8) | bytes[1])) / 800.0) - 40;
  *value2 = ((((bytes[2] << 8) | bytes[3])) / 800.0) - 40;
}

void uint32_to_uint8_array(uint32_t value, uint8_t *array) {
  array[0] = (uint8_t)(value & 0xFF);
  array[1] = (uint8_t)((value >> 8) & 0xFF);
  array[2] = (uint8_t)((value >> 16) & 0xFF);
  array[3] = (uint8_t)((value >> 24) & 0xFF);
}


float left_filter(float measured_speed){

  // 2nd Butterworth order Low-pass filter (25 Hz cutoff)
  float b[] = {0.00094408, 0.00188817, 0.00094408};
  float a[] = {1.91122623, -0.91500257};

  left_measured_speed_buffer[0] = measured_speed;

  float temp = left_filtered_speed_buffer[0]; 
  left_filtered_speed_buffer[0] = a[0] * left_filtered_speed_buffer[0] + a[1] * left_filtered_speed_buffer[1] + b[0] * left_measured_speed_buffer[0] + b[1] * left_measured_speed_buffer[1] + b[2] * left_measured_speed_buffer[2];
  left_filtered_speed_buffer[1] = temp;
  left_measured_speed_buffer[2] = left_measured_speed_buffer[1];
  left_measured_speed_buffer[1] = left_measured_speed_buffer[0];

  return left_filtered_speed_buffer[0];
}


float right_filter(float measured_speed){

  // 2nd Butterworth order Low-pass filter (25 Hz cutoff)
  float b[] = {0.00094408, 0.00188817, 0.00094408};
  float a[] = {1.91122623, -0.91500257};

  right_measured_speed_buffer[0] = measured_speed;

  float temp = right_filtered_speed_buffer[0]; 
  right_filtered_speed_buffer[0] = a[0] * right_filtered_speed_buffer[0] + a[1] * right_filtered_speed_buffer[1] + b[0] * right_measured_speed_buffer[0] + b[1] * right_measured_speed_buffer[1] + b[2] * right_measured_speed_buffer[2];
  right_filtered_speed_buffer[1] = temp;
  right_measured_speed_buffer[2] = right_measured_speed_buffer[1];
  right_measured_speed_buffer[1] = right_measured_speed_buffer[0];

  return right_filtered_speed_buffer[0];
}
