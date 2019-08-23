#define IN1 7
#define IN2 6
#define EN 5
#define encoderA 2
#define encoderB 3

// #define RIGHT_SIDE
#define LEFT_SIDE

const float wheel_vel_max = 465; // rad/sec
const float PWM_wheel_min = 35;

const int cmd_buf_length = 2;
int buff_idx = 0;

uint8_t cmdBuffer[cmd_buf_length];

int state = 0;

// ================================================================
// ===              Encoder INTERRUPT ROUTINE                    ===
// ================================================================
volatile float encoderCount = 0.0;
const float encoderPulsePerRev = 600.0;

float oldCount = 0.0;

void initialEncoder(int enA, int enB) {
    pinMode(enA, INPUT);
    pinMode(enB, INPUT);

    //turn on pull-up register because sensors are open collector
    digitalWrite(enA, HIGH); 
    digitalWrite(enB, HIGH);
    
    attachInterrupt(digitalPinToInterrupt(enA), doEncoder, RISING);
}

#ifdef RIGHT_SIDE
void doEncoder() {
    if (digitalRead(encoderB) == HIGH) {
            encoderCount++;     //CW
    } else {
        encoderCount--;     //CCW
    }
}
#endif

#ifdef LEFT_SIDE
void doEncoder() {
    if (digitalRead(encoderB) == HIGH) {
            encoderCount--;     //CW
    } else {
        encoderCount++;     //CCW
    }
}
#endif 

// ================================================================
// ===              TIMER1 INTERRUPT ROUTINE                    ===
// ================================================================
const float timr1_freq = 20.0; //hz
const float Pi = 3.14159;
float vel = 0.0;
// float vel_old;
// float desired_vel;

// ISR(TIMER1_COMPA_vect){
//     #ifdef RIGHT_SIDE
//     vel = ((encoderCount - oldCount)/encoderPulsePerRev)*timr1_freq*2.0*Pi;
//     #endif

//     #ifdef LEFT_SIDE
//     vel = ((encoderCount - oldCount)/encoderPulsePerRev)*timr1_freq*2.0*Pi;
//     #endif

//     oldCount = encoderCount;
// }

void setup() {
    cli(); // stop interrupts

    // pinMode(IN1, OUTPUT);
    // pinMode(IN2, OUTPUT);
    // pinMode(EN, OUTPUT);

    // set timer1 interrupt at 20 hz
    // TCCR1A = 0;
    // TCCR1B = 0;
    // TCNT1 = 0;
    // OCR1A = (16000000/(256*timr1_freq)) - 1; //3124; // = (16*10^6)/(pres*freq) - 1
    // TCCR1B |= (1 << WGM12);  // turn on CTC mode
    // TCCR1B |= (1 << CS12); // set CS12 for 256 prescaler
    // TIMSK1 |= (1 << OCIE1A);

    initialEncoder(encoderA, encoderB);

    sei(); // start interrupt

    Serial.begin(115200);
}

void loop() {    
    switch (state) {
    case 0:   //wait for data
        while (Serial.available() > 0){
            if (buff_idx < cmd_buf_length) {
                cmdBuffer[buff_idx] = Serial.read();
                buff_idx++;
            } else {
                buff_idx = 0;
                state = 1;
                break;
            }
        }
        break;
        
    case 1: //excecute buffer data   [instruction, param1:direction, param2:low_byte, param3:highByte]
        switch (cmdBuffer[0]) {
        case 0:     //read vel
            if (cmdBuffer[1] == 0) {
                vel = (((encoderCount - oldCount)/encoderPulsePerRev)*timr1_freq*2.0*Pi)*100.0;

                if (vel >= 0) {
                    Serial.write(0);
                    Serial.write(((int)vel & 0xFF));
                    Serial.write(((int)vel >> 8));
                    
                } else {
                    Serial.write(1);
                    Serial.write((abs((int)vel) & 0xFF));
                    Serial.write((abs((int)vel) >> 8));
                }

            oldCount = encoderCount;

            }
            
            state = 0;    
            break;
        
        case 1:     //write vel
            // desired_vel = (float)(cmdBuffer[2] | (cmdBuffer[3] << 8));
            // motorDrive(desired_vel, cmdBuffer[1]);
            // if (cmdBuffer[1] == 0) { // forward
            //     desired_vel = (float)(cmdBuffer[2] | (cmdBuffer[3] << 8));
            //     motorDrive(desired_vel, 0);
            //     // ForwardDrive(cmdBuffer[2]);
            // } else {    // backward
            //     desired_vel = (float)(cmdBuffer[2] | (cmdBuffer[3] << 8));
            //     motorDrive(desired_vel, 1);
            //     // BackwardDrive(cmdBuffer[2]);
            // }
            state = 0;
            break;
        
        case 2:
            motorsStop();
            state = 0;
            break;

        default:
            break;
        }
    
    default:
        break;
    }

    if (cmdBuffer[0] == 1){
        if (cmdBuffer[1] == 0) {
            ForwardDrive(cmdBuffer[2]);
        } else {
            BackwardDrive(cmdBuffer[2]);
        }
    } 
   
    
}
void motorDrive(float desired_vel, int dir){
    int PWM = map(desired_vel, 0, wheel_vel_max, 0, 255);
    if (dir == 0){
        ForwardDrive(PWM);
    } else {
        BackwardDrive(PWM);
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


void ForwardDrive(int PWM) {
    #ifdef RIGHT_SIDE
        if (PWM != 0) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(EN, PWM);
        } else {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, HIGH);
            analogWrite(EN, 0);
        }
    #endif

    #ifdef LEFT_SIDE
        if (PWM != 0) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(EN, PWM);
        } else {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, HIGH);
            analogWrite(EN, 0);
        }
    #endif   
}

void BackwardDrive(int PWM) {
    #ifdef RIGHT_SIDE
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(EN, PWM);
    #endif

    #ifdef LEFT_SIDE
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(EN, PWM);
    #endif
}

void motorsStop() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(EN, 0);
}



