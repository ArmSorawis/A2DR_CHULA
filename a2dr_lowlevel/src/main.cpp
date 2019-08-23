#include <Arduino.h>
#include <math.h>

// Robot Parameters
#define IN1_R 8
#define IN2_R 13
#define EN_R 9
#define IN1_L 12
#define IN2_L 11
#define EN_L 10

const float wheel_seperation = 0.342;  // meter
const float wheel_radius = 0.075; // meter
const float wheel_vel_max = 4.0; // rad/sec
const float PWM_wheel_min = 35;

// Timer Interrupt Variables
const float timr1_freq = 20.0; //hz
const float Pi= 3.14159;

// Package Variables
#define ID 1
#define SEND_VEL 1
#define VEL_LENGTH 6
byte receive_buffer[4]; //Dir_linear, linear_vel, Dir_angular, angular_vel
byte send_vel_buffer[VEL_LENGTH]; //Dir_R, V_R_lowbyte, V_R_highbyte, Dir_L, V_L_lowbyte, V_L_highbyte
byte start_package[3] = {255, 255, ID}; //start_byte, start_bye, ID

// Control Variables
#define TOL 0.1
float linear_vel = 0.0;
float angular_vel = 0.0;
float v_right = 0.0;
float v_left = 0.0;

float vc_r, vc_l;
float er_l, er_r, sum_er_l, sum_er_r, old_er_l, old_er_r, u_l, u_r;

uint8_t smach = 1; //State Machine
uint8_t state_serial = 1;
byte buff;
byte length;
uint8_t idx;

void ForwardDrive(uint8_t PWM, uint8_t motor){
  if (motor == 0) {
        if (PWM != 0) {
            digitalWrite(IN1_R, HIGH);
            digitalWrite(IN2_R, LOW);
            analogWrite(EN_R, PWM);
        } else {
            digitalWrite(IN1_R, HIGH);
            digitalWrite(IN2_R, HIGH);
            analogWrite(EN_R, 0);
        }
    } else {
        if (PWM != 0) {
            digitalWrite(IN1_L, LOW);
            digitalWrite(IN2_L, HIGH);
            analogWrite(EN_L, PWM);
        } else {
            digitalWrite(IN1_L, HIGH);
            digitalWrite(IN2_L, HIGH);
            analogWrite(EN_L, 0);
        }
    }   
}

void BackwardDrive(uint8_t PWM, uint8_t motor) {
  if (motor == 0) {
      digitalWrite(IN1_R, LOW);
      digitalWrite(IN2_R, HIGH);
      analogWrite(EN_R, PWM);
  } else {
      digitalWrite(IN1_L, HIGH);
      digitalWrite(IN2_L, LOW);
      analogWrite(EN_L, PWM);
  }
}

void motorDrive(float vel, uint8_t motor) {
  uint8_t PWM = map(abs(vel), 0, wheel_vel_max, 0, 255);
  if (vel >= 0){
      ForwardDrive(PWM, motor);
  } else {
      BackwardDrive(PWM, motor);
  }
}

void robot_drive(float linear_vel, float angular_vel) {
  v_right = (2.0*linear_vel + wheel_seperation*angular_vel)/(2.0*wheel_radius);
  v_left = (2.0*linear_vel - wheel_seperation*angular_vel)/(2.0*wheel_radius);

  motorDrive(v_right, 0);
  motorDrive(v_left, 1);
}

float kp = 0.1;
float ki = 0.0;
float kd = 0.0;


void velocity_control(float v_cr, float v_cl) {
    er_l = v_left - v_cl;
    er_r = v_right - v_cr;

    sum_er_l = sum_er_l + er_l;
    sum_er_r = sum_er_r + er_r;

    u_l = (kp * er_l) + (ki * sum_er_l) + (kd * (er_l - old_er_l));
    u_r = (kp * er_r) + (ki * sum_er_r) + (kd * (er_r - old_er_r));

    if (abs(er_l) > TOL) {
      motorDrive(v_left + u_l, 1);
      old_er_l = er_l;
    }
    if (abs(er_r) > TOL) {
      motorDrive(v_right + u_r, 0);
      old_er_r = er_r;
    } 
}

void setup() {
  cli(); // stop interrupts

  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(EN_L, OUTPUT);

  // connect to PC
  Serial.begin(115200);

  // connect to slaves
  Serial1.begin(115200);
  Serial2.begin(115200);

  Serial.flush();
  Serial1.flush();
  Serial2.flush();
  
  // set timer1 interrupt at 20 hz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = (16000000/(256*timr1_freq)) - 1; //3124; // = (16*10^6)/(pres*freq) - 1
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS12); // set CS12 for 256 prescaler
  TIMSK1 |= (1 << OCIE1A);

  for (uint8_t i = 0 ; i < VEL_LENGTH ; i++){
    send_vel_buffer[i] = 0;
  }

  sei(); // start interrupt
}

void loop() {
  switch (smach) {
  case 1: // wait for connected to slaves
    while (!Serial1 && !Serial2) { 
      // Serial.println("wait");
    }
    smach = 2;
    break;

  case 2: // Initialize IMU

    smach = 3;
    break;

  case 3:
    while (Serial.available() > 0) {
      buff = Serial.read();

      switch (state_serial) {
      case 1:
        if (buff == 255) {
          state_serial = 2;
        } 
        break;

      case 2:
        if (buff == 255) {
          state_serial = 3;
        } else {
          state_serial = 1;
        }
        break;

      case 3:
        if (buff == ID) {
          state_serial = 4;
        } else {
          state_serial = 1;
        }
        break;

      case 4:
        length = buff;
        state_serial = 5;
        break;

      case 5:
        receive_buffer[idx] = buff;
        idx++;
        if (idx == length){
          if (receive_buffer[0] == 0) {
            linear_vel = (float)receive_buffer[1];  
          } else {
            linear_vel = -(float)receive_buffer[1];
          }

          if (receive_buffer[2] == 0) {
            angular_vel = (float)receive_buffer[3];
          } else {
            angular_vel = -(float)receive_buffer[3];
          }
          
          robot_drive(linear_vel/100.0, angular_vel/100.0);
      
          sum_er_l = 0.0;
          sum_er_r = 0.0;

          state_serial = 1;
          idx = 0;
          Serial.flush();
        }
        break;
      }
    }
  }
}

void send_vel(uint8_t buff[6]) {
  for (uint8_t i = 0 ; i < 3 ; i++) { // send 255 255 ID
    Serial.write(start_package[i]);
  }
  Serial.write(SEND_VEL); // send instruction
  Serial.write(VEL_LENGTH); // send length
  for (int i = 0 ; i < VEL_LENGTH ; i++) { // send datas
    Serial.write(buff[i]);
  }
}

ISR(TIMER1_COMPA_vect){
    // read v_wheel from slave
  if (smach == 3){
      Serial1.write(0);
      Serial1.write(0);
      
      send_vel_buffer[0] = Serial1.read();
      send_vel_buffer[1] = Serial1.read();
      send_vel_buffer[2] = Serial1.read();
      
      Serial2.write(0);
      Serial2.write(0);
      
      send_vel_buffer[3] = Serial2.read();
      send_vel_buffer[4] = Serial2.read();
      send_vel_buffer[5] = Serial2.read();
      
      send_vel(send_vel_buffer);

      if (send_vel_buffer[0] == 0) {
        vc_l = send_vel_buffer[1] || (send_vel_buffer[2] << 8);
      } else {
        vc_l = -(send_vel_buffer[1] || (send_vel_buffer[2] << 8));
      }

      if (send_vel_buffer[3] == 0) {
        vc_r = send_vel_buffer[4] || (send_vel_buffer[5] << 8);
      } else {
        vc_r = -(send_vel_buffer[4] || (send_vel_buffer[5] << 8));
      }

      // velocity_control(vc_r/100.0, vc_l/100.0);

  }
}


