#include <Encoder.h>
#include <cmath>
#define PI 3.1415926535897932384626433832795


Encoder encl(25, 26);
Encoder encr(30, 31);

// ----------- PIN -------------

int LEVEL_SHIFTER = 2;         // Level-shifter pin
const uint8_t C_l = 14;        // IN1-A
const uint8_t D_l = 15;        // IN2-A
const uint8_t PWM_l = 22;      // Enable A
const uint8_t CURRENT_l = 40;  // Sense A
const uint8_t C_r = 3;         // IN1-B
const uint8_t D_r = 4;         // IN2-B
const uint8_t PWM_r = 23;      // Enable B
const uint8_t CURRENT_r = 41;  // Sense B


// ---- GLOBAL VARIABLES -----

// Control related variables
int previous_time, current_time = 0;
int *previous_pos_l, *previous_pos_r;
float *previous_error_l, *previous_error_r;
float previous_voltage = 0;
float *integral_term_l;
float *integral_term_r;
float *previous_encl, *previous_encr;

// Arrays with previous values of omega measured/filtered used to fliter measured speed
float *omega_rfilt, *omega_lfilt;
float *omega_lmes, *omega_rmes;
float *omega_rref, *omega_lref;
float *x, *y, *theta;
float *x_ref, *y_ref;

// Physical properties
float l = 0.25;  // Distance between wheels [m]
int cpr = 2048;  // Count per revolution of encoder.
float mass = 0.1;           //minibot mass [kg]
float wheel_radius = 0.03;  //wheel radius [m]
float max_voltage = 24;
float max_current = 10;
float K = 1;  //power electronics gain
float k_phi = 3.962 * 1e-3 * 2 * PI / 60;
float R_a = 5.84;  //armature resistance
float K_v = 1.26e-3;
float J_r = mass / 2 * wheel_radius * wheel_radius / 2;  //rotor inertia
float tau_m = J_r / K_v;

float A; // Omega reference amplitude


void setup() {
  Serial.begin(2400);

  // Motor output pins
  pinMode(C_l, OUTPUT);
  pinMode(D_l, OUTPUT);
  pinMode(PWM_l, OUTPUT);

  pinMode(C_r, OUTPUT);
  pinMode(D_r, OUTPUT);
  pinMode(PWM_r, OUTPUT);

  // Current input pins
  pinMode(CURRENT_l, INPUT);
  pinMode(CURRENT_r, INPUT);

  // Enable the level shifter
  pinMode(LEVEL_SHIFTER, OUTPUT);
  digitalWrite(LEVEL_SHIFTER, HIGH);

  // Measured speed
  omega_lmes = (float *)calloc(3, sizeof(float));
  omega_rmes = (float *)calloc(3, sizeof(float));

  // Filtered measured speed
  omega_lfilt = (float *)calloc(2, sizeof(float));
  omega_rfilt = (float *)calloc(2, sizeof(float));

  // Integral terms for integral control
  integral_term_l = (float *)calloc(1, sizeof(float));
  integral_term_r = (float *)calloc(1, sizeof(float));

  // Previous position in order to compute motor's speed.
  previous_pos_l = (int *)calloc(1, sizeof(int));
  previous_pos_r = (int *)calloc(1, sizeof(int));
  ;

  previous_error_l = (float *)calloc(1, sizeof(float));
  previous_error_r = (float *)calloc(1, sizeof(float));

  previous_encl = (float *)calloc(1, sizeof(float));
  previous_encr = (float *)calloc(1, sizeof(float));

  // Reference speed
  omega_lref = (float *)malloc(sizeof(float));
  omega_rref = (float *)malloc(sizeof(float));

  x = (float *)calloc(1, sizeof(float));
  y = (float *)calloc(1, sizeof(float));
  theta = (float *)calloc(1, sizeof(float));

  x_ref = (float *)calloc(1, sizeof(float));
  y_ref = (float *)calloc(1, sizeof(float));

  A = 30;

}

void loop() {

  current_time = micros();
  if (current_time - previous_time > 500) {

    float time_interval = (current_time - previous_time) * 1e-6;

    float *V_r = (float *)malloc(sizeof(float));
    float *V_l = (float *)malloc(sizeof(float));

    // Read position error on DE0
    readPosition(x_ref, y_ref);

    Serial.print(*x_ref);
    Serial.print(" ");
    Serial.print(*y_ref);
    Serial.print(" ");

    computePosition(x, y, theta, previous_encl, previous_encr);

    // Compute reference speed for both motors from position error
    positionControl(*x-*x_ref, *y-*y_ref, *theta, omega_lref, omega_rref);

    //*omega_lref = A  * cos(2*PI*current_time*1e-6);;
    //*omega_rref = -A  * cos(2*PI*current_time*1e-6);;

    if (Serial.available() > 0) {
      // Lit la donnée entrante et met à jour la variable
      A = Serial.parseInt();
      // Vide le tampon de la communication série
      while (Serial.available() > 0) {
        Serial.read();
      }
    }

    // Control motors
    speedControl(*omega_rref, encr, previous_pos_r, time_interval, integral_term_r, PWM_r, C_r, D_r, omega_rmes, omega_rfilt, previous_error_r, V_r);
    speedControl(*omega_lref, encl, previous_pos_l, time_interval, integral_term_l, PWM_l, C_l, D_l, omega_lmes, omega_lfilt, previous_error_l, V_l);

    //Plot

    /*Serial.print(*omega_lref);
    Serial.print(" ");
    Serial.print(*omega_rref);
    Serial.print(" ");
    Serial.print(omega_lfilt[0]);
    Serial.print(" ");
    Serial.println(omega_rfilt[0]);
    Serial.print(" ");  
    Serial.print(*V_l);
    Serial.print(" ");  
    Serial.print(*V_r);
    Serial.print(" ");
    Serial.print((*V_l - k_phi*omega_lfilt[0]) / R_a);
    Serial.print(" ");  
    Serial.print((*V_r - k_phi*omega_rfilt[0]) / R_a);
    Serial.print(" ");
    Serial.print(analogRead(CURRENT_l) / 1024);
    Serial.print(" ");  
    Serial.println(analogRead(CURRENT_r) / 1024);*/


    //Free memory
    free(V_r); 
    free(V_l);
    previous_time = current_time;
  }
}


void readPosition(float *x_ref, float *y_ref) {
  // Lis en SPI ou CAN
  *x_ref = 3.0;
  *y_ref = 7.0;
}


void computePosition(float *x, float *y, float *theta, float *previous_encl, float *previous_encr) {
  // The aim of this function is to compute dx, dy and dtheta based on measured position coming from encoders (no slip, lidar later)
  float enc_l = encl.read();
  float enc_r = encr.read();
  float dl_l = ((enc_l-*previous_encl) * wheel_radius * 2*PI) / (19*cpr);
  float dl_r = ((enc_r-*previous_encr) * wheel_radius * 2*PI) / (19*cpr);
  float dl = (dl_l + dl_r) / 2;
  float dtheta = (dl_r - dl_l) / l;
  *x += dl * cos(*theta + dtheta);
  *y += dl * sin(*theta + dtheta);
  *theta += dtheta;
  if (*theta > PI) { *theta -= 2*PI; }
  else if (*theta < -PI) { *theta += 2*PI; }
  Serial.print(*x);
  Serial.print(" ");
  Serial.print(*y);
  Serial.print(" ");
  Serial.println(*theta);
  *previous_encl = enc_l;
  *previous_encr = enc_r;
}


void positionControl(float dx, float dy, float theta, float *omega_lref, float *omega_rref) {

  // Position control variables
  float krho = 30;
  float kalpha = 50;

  // Variable change
  float rho = sqrt(pow(dx, 2) + pow(dy, 2));
  float alpha = atan2(dy, dx) + PI - theta;

  if (abs(alpha) > PI/2) {
    krho = 0;
  } else {
    krho = krho*cos(alpha);
  }

  // Compute speed reference
  float omega_ref = kalpha * alpha; // + kbeta * beta;
  float v_ref = rho * krho;

  if (rho < 0.02) {
    v_ref = 0;
    omega_ref = 0;
  }

  // Set speed reference
  *omega_lref = v_ref - omega_ref * l;
  *omega_rref = v_ref + omega_ref * l;
}


void speedControl(float omega_ref, Encoder enc, int *previous_pos, float time_interval, float *integral_term, int pwm, int in1, int in2, float *omega_mes, float *omega_filt, float *previous_error, float *V) {
  /*
    inputs: omega_ref: motor speed instruction we give to the controller
            enc: Encoder object reference
            previous_pos: integral of the previous error
            time_interval: time difference between two control iteration.
            integral_term: Integral of the previous error
            PWM/in1/in2: pwm and motor pins.
            omega_mes: Adress of measured speed
            omega_filt: Adress of filtered measured speed 
    */

  //read encoders
  int pos = enc.read();

  //compute speed
  omega_mes[0] = (pos - *previous_pos) / time_interval * 2 * PI / (cpr * 19);

  // 1st order Low-pass filter (25 Hz cutoff)
  //float b[] = { 0.03045903, 0.03045903, 0 };
  //float a[] = { 0.93908194, 0 };

  // 2nd Butterworth order Low-pass filter (25 Hz cutoff)
  float b[] = {0.00094408, 0.00188817, 0.00094408};
  float a[] = {1.91122623, -0.91500257};

  float temp = omega_filt[0]; 
  omega_filt[0] = a[0] * omega_filt[0] + a[1] * omega_filt[1] + b[0] * omega_mes[0] + b[1] * omega_mes[1] + b[2] * omega_mes[2];
  omega_filt[1] = temp;
  omega_mes[2] = omega_mes[1];
  omega_mes[1] = omega_mes[0];

  // Speed control
  *V = PI_motor_controller(*omega_filt, omega_ref, integral_term, time_interval, previous_error);

  // Map voltage into pwm value
  float pwmVal = min(255, *V * 255 / 24);

  // Motor setting
  int dir = 1;
  if (*V > 0) {
    dir = -1;
  }
  setMotor(dir, fabs(pwmVal), pwm, in1, in2);
  *previous_pos = pos;
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  /*
    inputs: dir: Rotation direction -1 or 1.
            pwmVal: Voltage modulation value from 0 to 255.
            pwm: Pwm pin
            in1/in2: Motor pins
    */

  analogWrite(pwm, pwmVal);  // Motor speed

  if (dir == -1) {
    // Turn one way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (dir == 1) {
    // Turn the other way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


float PI_motor_controller(float omega_mes, float omega_ref, float *integral_term, float time_interval, float *previous_error) {
  /*
    inputs: omega_in: actual speed of the motor, measured by the motor encoder
            omega_ref: motor speed instruction we give to the controller
            integral_term: integral of the previous error
            time_interval: time of the last controller calculution [s]

    output: output_voltage: average motor voltage
    */

  float controller_time_constant = 1;  //defines the preciseness of the controller
  float error = omega_ref - omega_mes;

  // Predictive term
  // Made by inverting th model of DC machine
  // Assuming omega constant <-> mechanical time constant smaller than electrical one.
  // av'(t) + bv(t) = w
  // v(0) = previous_voltage
  // Subject to modelling error -> compensated with parrallel PID
  float a = J_r * R_a / k_phi;
  float b = K_v * R_a / k_phi + k_phi;
  float output_voltage = (previous_voltage - error / b) * exp(-b / a * time_interval) + error / b;
  // Uncomment to activate
  output_voltage = 0;

  // PID controller
  float Ki = (R_a * K_v) / (K * k_phi * controller_time_constant);
  float Kp = tau_m * Ki;
  float Kd = Ki / 1000 * controller_time_constant;
  *integral_term += error * time_interval;

  //anti-windup
  if (*integral_term > max_voltage / Ki) {
    *integral_term = max_voltage / Ki;
  } else if (*integral_term < -max_voltage / Ki) {
    *integral_term = -max_voltage / Ki;
  }

  float dedt = (error - *previous_error) / (time_interval);
  output_voltage += K * (Kp * error + Ki * (*integral_term) + Kd * dedt + k_phi * omega_mes / K);

  // Voltage limiter
  if (output_voltage > max_voltage) {
    output_voltage = max_voltage;
  } else if (output_voltage < -max_voltage) {
    output_voltage = -max_voltage;
  }

  // Current limiter
  if ((output_voltage - k_phi*omega_mes) / R_a > max_current) {
    output_voltage = max_current * R_a + k_phi * omega_mes;
  } else if ((output_voltage - k_phi*omega_mes) / R_a < -max_current) {
    output_voltage = -max_current * R_a + k_phi * omega_mes;
  }

  *previous_error = error;
  //previous_voltage = output_voltage;

  return output_voltage;
}

