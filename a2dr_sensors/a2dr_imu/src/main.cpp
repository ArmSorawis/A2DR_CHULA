

#include "mbed.h"
#include <stdio.h>
#include "stm32f4_for_mpu.h"
// #include "hcsr04.h"

#define ID 0x02
#define readStrc 0x02
#define len 0x06

//Initail Pin
// #define trigPinL D10
// #define echoPinL D9
// #define trigPinR D7
// #define echoPinR D6
#define interruptPinIMU D3
#define nanoTx PA_9 //D1
#define nanoRx PA_10 //D0

//set Pin
// HCSR04 ultraL(trigPinL, echoPinL); // TRIGGER , ECHO
// HCSR04 ultraR(trigPinR, echoPinR); // TRIGGER , ECHO
// InterruptIn newData(interruptPinIMU); //interrupt
DigitalIn newData(interruptPinIMU); //interrupt


//set IMU variable
extern struct hal_s hal;
Timer t;

//set serial communication
Serial serial(USBTX, USBRX, 115200);
Serial nano(nanoTx, nanoRx, 115200);
Ticker tickSend;


//***************************************************************//
//                             Variable                          //
//***************************************************************//

float yaw = 0;
int yawByte[2] = {0};
long data[4] = {0};
int8_t accuracy = 0;
int32_t timestamp = 0;

int count_imu_reset = 0;

uint8_t buffer[2] = {0};
uint8_t distanceL = 0;
uint8_t distanceR = 0;
int joe = 0;


//***************************************************************//
//                            Function                           //
//***************************************************************//

void CallBack()
{
  // interupt data is ready
  hal.new_gyro = 1;
}//CallBack()

//__________________________________________________//

void sendData()
{
  // distanceL = ultraL.distance();
  // distanceR = ultraR.distance();
  uint8_t package[10] = { 0xff, 0xff, ID, len, readStrc, accuracy, yawByte[0], yawByte[1], buffer[0], buffer[1] };
  for(int i = 0; i < 10; i++){
    serial.putc(package[i]);
  }
}// sendData()

//__________________________________________________//

void GetPackage()
{
int lenOfbuffer = 2;
 if (nano.getc() == 0xFF)
  { // check first header
    if (nano.getc() == 0xFF)
    { // check second header
      if (nano.getc() == ID)
      { // check ID
        for (int index = 0; index < lenOfbuffer; index++)
        {
          buffer[index] = nano.getc();
        } //for index

        sendData();

      } //if third_package = ID
    }   //if second_buf = 0xFF
  }     //if first_buf = 0xFF
} //GetPackage

void test(){
  joe = nano.getc();
}

//__________________________________________________//
//***************************************************************//
//                              MAIN                             //
//***************************************************************//

int main(){
  
  // tickSend.attach(&sendData, 0.05);//20 Hz
  nano.attach(&GetPackage);
  // nano.attach(&test);

  // t.start();
  // newData.rise(&CallBack);

  // initial IMU
  MPU9250_Initial();

  while (1)
  {
    if (newData == 1)
    {
        CallBack();
    }
    
    if (MPU9250_Process())
    {
      inv_execute_on_data();
      inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp);

      // serial.printf("yaw = %f  acc = %d // %d : %d\n", (float)data[2] / 65536, accuracy, buffer[0], buffer[1]);
      // serial.printf("Roll:%f Pitch%f Yaw%f Acc %d \n", (float)data[0] / 65536, (float)data[1] / 65536, (float)data[2] / 65536, accuracy);
      // inv_get_sensor_type_compass(data, &accuracy, (inv_time_t *)&timestamp);
      // serial.printf("Mx:%f My%f Mz%f Acc %d\r\n", (float)data[0] / 65536, (float)data[1] / 65536, (float)data[2] / 65536, accuracy);

        yaw = (float)data[2] / 65536;

        if (yaw < 0)
        {
          yawByte[0] = abs(yaw);
          yawByte[1] = 0;
        } // if yaw < 0
        else
        {
          yawByte[0] = 0;
          yawByte[1] = yaw;
        } // else
        // serial.printf("%f ==> [ %d, %d ] = %d \n",yaw, yawByte[0],yawByte[1],yawByte[0]+yawByte[1]);

      } // if


  } //while
} //main
//*******************************************************************************************************************************************//
