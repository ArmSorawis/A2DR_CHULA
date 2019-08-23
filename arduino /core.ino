#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <math.h>

// ================================================================
// ===                Define and Initial Robot                  ===
// ================================================================

#define IN1_R 8
#define IN2_R 13
#define EN_R 9
#define IN1_L 12
#define IN2_L 11
#define EN_L 10

uint8_t smach = 1;
const float wheel_seperation = 0.342;  // meter
const float wheel_radius = 0.075; // meter
const float wheel_vel_max = 4.65; // rad/sec
const float PWM_wheel_min = 35;

int cmdBuffer[3] = {1, 0, 255};

// ================================================================
// ===              Rosserial CallBack Function                 ===
// ================================================================

const float Kp = 1.0;
const float Ki = 0.0;
const float Kd = 0.0;

volatile float delta_vel_right;
volatile float delta_vel_left;
volatile int delta_PWM_left;
volatile int delta_PWM_right;
volatile int input_PWM_right;
volatile int input_PWM_left;
volatile float error_right = 0;
volatile float error_left = 0;
volatile float sum_error_right = 0;
volatile float sum_error_left = 0;
volatile float old_error_right = 0;
volatile float old_error_left = 0;

float desired_v_right;
float desired_v_left;
int desired_PWM_right;
int desired_PWM_left;

void cmd_vel_CB( const geometry_msgs::Twist& cmd_vel) {
    desired_v_right = (2.0*cmd_vel.linear.x + wheel_seperation*cmd_vel.angular.z)/(2.0*wheel_radius);
    desired_v_left = (2.0*cmd_vel.linear.x - wheel_seperation*cmd_vel.angular.z)/(2.0*wheel_radius);

    // desired_PWM_left = map(desired_v_left, -wheel_vel_max, wheel_vel_max, -255, 255);
    // desired_PWM_right = map(desired_v_right, -wheel_vel_max, wheel_vel_max, -255, 255);

    // sum_error_left = 0;
    // sum_error_right = 0;

    motorDrive(desired_v_right, 0);
    motorDrive(desired_v_left, 1);   
}

// ================================================================
// ===               Define Rosserial Handling                  ===
// ================================================================

ros::NodeHandle nh;
// sensor_msgs::Imu imu_msg;
std_msgs::Float32 v_wheel_right_msg, v_wheel_left_msg, imu_msg;
std_msgs::Int8 state_msg;

ros::Publisher pub_vw_r("v_wheel_right", &v_wheel_right_msg), pub_vw_l("v_wheel_left", &v_wheel_left_msg), pub_imu("imu", &imu_msg);
ros::Publisher pub_state("state", &state_msg);

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", cmd_vel_CB);

// ================================================================
// ===               Define and Initial MPU6050                 ===
// ================================================================

// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     #include "Wire.h"
// #endif

// MPU6050 mpu;
// #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// // MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// int sequence = 1;
// float yaw_last = 0;
// float yaw = 0;

// ================================================================
// ===              TIMER1 INTERRUPT ROUTINE                    ===
// ================================================================
const float timr1_freq = 20.0; //hz
const float Pi= 3.14159;
uint8_t dirR = 0;
uint8_t velR_lowbyte = 0;
uint8_t velR_highbyte = 0;

uint8_t dirL = 0;
uint8_t velL_lowbyte = 0;
uint8_t velL_highbyte = 0;

float velR;
float velL;


ISR(TIMER1_COMPA_vect){
    // read v_wheel from slave
    if (smach == 3){
        Serial1.write(0);
        Serial1.write(0);
        
        dirL = Serial1.read();
        velL_lowbyte = Serial1.read();
        velL_highbyte = Serial1.read();
        if (dirL == 0){
            velL = velL_lowbyte | (velL_highbyte << 8);
        } else {
            velL = -(velL_lowbyte | (velL_highbyte << 8));
        }
        
        Serial2.write(0);
        Serial2.write(0);
        
        dirR = Serial2.read();
        velR_lowbyte = Serial2.read();
        velR_highbyte = Serial2.read();
        if (dirR == 0){
            velR = velR_lowbyte | (velR_highbyte << 8);
        } else {
            velR = -(velR_lowbyte | (velR_highbyte << 8));
        }
        
        v_wheel_left_msg.data = velL;
        v_wheel_right_msg.data = velR;
        imu_msg.data = desired_v_right*100.0;

        // pub_imu.publish( &imu_msg );
        pub_vw_r.publish( &v_wheel_right_msg );
        pub_vw_l.publish( &v_wheel_left_msg );

        // // close loop velocity control 
        // error_left = desired_v_left - velL/100;
        // error_right = desired_v_right - velR/100;

        // if (abs(error_left) > 0.2) {
        //     sum_error_left = sum_error_left + error_left;
        //     delta_vel_left = Kp*error_left + Ki*sum_error_left + Kd*(error_left - old_error_left);
        //     delta_PWM_left = map(delta_vel_left, -wheel_vel_max, wheel_vel_max, -255, 255);
        //     input_PWM_left = desired_PWM_left + delta_PWM_left;

        //     if (input_PWM_left >= 0) {
        //         ForwardDrive(input_PWM_left, 1);
        //     } else {
        //         BackwardDrive(-input_PWM_left, 1);
        //     }

        //     desired_PWM_left = input_PWM_left;

        // }
        // old_error_left = error_left;
        
        // if  (abs(error_right) > 0.2) {
        //     sum_error_right = sum_error_right + error_right;
        //     delta_vel_right = Kp*error_right + Ki*sum_error_right + Kd*(error_right - old_error_right);
        //     delta_PWM_right = map(delta_vel_right, -wheel_vel_max, wheel_vel_max, -255, 255);
        //     input_PWM_right = desired_PWM_right + delta_PWM_right;

        //     if (input_PWM_right >= 0) {
        //         ForwardDrive(input_PWM_right, 0);
        //     } else {
        //         BackwardDrive(-input_PWM_right, 0);
        //     }

        //     desired_PWM_right = input_PWM_right;
        // }
        // old_error_right = error_right;
        

        
    }
}

// ================================================================
// ===       INTERRUPT MPU6050 DETECTION ROUTINE                ===
// ================================================================

// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {
//     mpuInterrupt = true;
// }

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    cli(); // stop interrupts
    
    // //join I2C bus (I2Cdev library doesn't do this automatically)
    // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    //     Wire.begin();
    //     Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //     Fastwire::setup(400, true);
    // #endif

    pinMode(IN1_R, OUTPUT);
    pinMode(IN2_R, OUTPUT);
    pinMode(EN_R, OUTPUT);
    pinMode(IN1_L, OUTPUT);
    pinMode(IN2_L, OUTPUT);
    pinMode(EN_L, OUTPUT);

    // connect to slaves
    Serial1.begin(57600);
    Serial2.begin(57600);

    // set timer1 interrupt at 20 hz
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = (16000000/(256*timr1_freq)) - 1; //3124; // = (16*10^6)/(pres*freq) - 1
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS12); // set CS12 for 256 prescaler
    TIMSK1 |= (1 << OCIE1A);

    nh.initNode();  
    nh.advertise(pub_vw_r);
    nh.advertise(pub_vw_l);
    nh.advertise(pub_imu);
    nh.subscribe(sub_cmd_vel);
    sei();  // start interrupt
}

// ================================================================
// ===                     Main Program                         ===
// ================================================================

void loop() {   
    switch (smach) {
    case 1: // wait for rosserial connected
        while (!nh.connected()) {
            nh.spinOnce();
        }
        nh.loginfo("rosserial connected");

        while (!Serial1 && !Serial2) {
            nh.logwarn("wait for serial slaves port to connect");
        }

        nh.loginfo("connected to slaves");
        smach = 2;
        break;

    case 2:
        // nh.loginfo("Initializing IMU device");
        // mpu.initialize();
        // pinMode(INTERRUPT_PIN, INPUT);
        
        // while (!mpu.testConnection()){
        //     nh.logwarn("connecting mpu");
        // }
        
        // nh.loginfo("Initializing DMP");
        // devStatus = mpu.dmpInitialize();

        // mpu.setXGyroOffset(220);
        // mpu.setYGyroOffset(76);
        // mpu.setZGyroOffset(-85);
        // mpu.setZAccelOffset(1788);

        // if (devStatus == 0) {
        //     nh.loginfo("Enabling DMP");
        //     mpu.setDMPEnabled(true);

        //     nh.loginfo("Enabling interrupt detection");
        //     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        //     mpuIntStatus = mpu.getIntStatus();

        //     dmpReady = true;
        //     packetSize = mpu.dmpGetFIFOPacketSize();

        //     smach = 3;
        // } else {
        //     nh.logerror("DMP Initialization failed");
        //     smach = 0;
        // }
        smach = 3;
        break;
    case 3:
        while (nh.connected()) {
            // while (!mpuInterrupt && fifoCount < packetSize) {
            //     if (mpuInterrupt && fifoCount < packetSize) {
            //         fifoCount = mpu.getFIFOCount();
            //         }  
            // }

            // // reset interrupt flag and get INT_STATUS byte
            // mpuInterrupt = false;
            // mpuIntStatus = mpu.getIntStatus();

            // // get current FIFO count
            // fifoCount = mpu.getFIFOCount();

            // // check for overflow (this should never happen unless our code is too inefficient)
            // if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
            //     // reset so we can continue cleanly
            //     mpu.resetFIFO();
            //     fifoCount = mpu.getFIFOCount();
            //     nh.logwarn("FIFO overflow!");

            // // otherwise, check for DMP data ready interrupt (this should happen frequently)
            // } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
            //     // wait for correct available data length, should be a VERY short wait
            //     while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            //     // read a packet from FIFO
            //     mpu.getFIFOBytes(fifoBuffer, packetSize);
                
            //     // track FIFO count here in case there is > 1 packet available
            //     // (this lets us immediately read more without waiting for an interrupt)
            //     fifoCount -= packetSize;

            //     // display Euler angles in degrees
            //     mpu.dmpGetQuaternion(&q, fifoBuffer);
            //     mpu.dmpGetGravity(&gravity, &q);
            //     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                
            // }
            nh.spinOnce();
            delay(1);
        }
        smach = 1;
        break;
        
    default:
        nh.logerror("Program or Devices Error Pls Reconnected....");
        return;
    }
}



void motorDrive(float desired_vel, uint8_t motor){
    int PWM = map(abs(desired_vel), 0, wheel_vel_max, 0, 255);
    if (desired_vel >= 0){
        ForwardDrive(PWM, motor);
    } else {
        BackwardDrive(PWM, motor);
    }

}


int mapVelocity2PWM(float actual_velocity) {
    int PWM = map(abs(actual_velocity), 0, wheel_vel_max, 0, 255);
    
    if (actual_velocity > 0) {
        if (PWM <= PWM_wheel_min){
            return PWM_wheel_min;
        }
        else {
            return PWM;
        }
    }
    else if (actual_velocity == 0) {
        return 0;
    }
    else {
        if (PWM <= PWM_wheel_min) {
            return -PWM_wheel_min;
        }
        else {
            return -PWM;
        }  
    }
}


void ForwardDrive(uint8_t PWM, uint8_t motor) {
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

void motorsStop() {
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, HIGH);
    analogWrite(EN_R, 0);

    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, HIGH);
    analogWrite(EN_L, 0);


}