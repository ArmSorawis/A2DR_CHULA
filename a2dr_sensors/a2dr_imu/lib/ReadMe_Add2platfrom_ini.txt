[env:nucleo_f411re]
platform = ststm32
board = nucleo_f411re
framework = mbed
editer = vs code
copy folder MPU9250 to lib folder in workspace
;*******************add below to platform.ini *******************;

build_flags = 
    -D EMPL
    -D EMPL_TARGET_STM32F4
    -D MPU9250
    -D USE_DMP

    -I"$PROJECT_DIR/lib/MPU9250/eMPL-hal"
    -I"$PROJECT_DIR/lib/MPU9250/mllite"
    -I"$PROJECT_DIR/lib/MPU9250/mpl"
    -I"$PROJECT_DIR/lib/MPU9250"
    -I"$PROJECT_DIR/lib/MPU9250/driver/eMPL"
    -I"$PROJECT_DIR/lib/MPU9250/driver/include"
    -L"$PROJECT_DIR/lib/MPU9250/mpl/"
    -l"liblibmplmpu.a"
    
    
#**************************** example code ******************************#

// this code used D14 and D15 for I2C if you want to change pin please go to stm32f4_for_mpu.cpp
//and change pin at the the top of code 

#include "mbed.h"
#include "stm32f4_for_mpu.h"

extern struct hal_s hal;
Timer t;
Serial serial(USBTX, USBRX);

//set interupt pin
InterruptIn newData(D13);

void CallBack()
{
  // interupt data is ready
  hal.new_gyro = 1;
}
int main()
{
  t.start();
  newData.rise(&CallBack);
  serial.baud(115200);
  // initial IMU
  MPU9250_Initial();
  while (1)
  {
    if (MPU9250_Process())
    {
      inv_execute_on_data();
      long data[4] = {0};
      int8_t accuracy = 0;
      int32_t timestamp = 0;

      inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp);
      serial.printf("Roll:%f Pitch%f Yaw%f Acc %d", (float)data[0] / 65536, (float)data[1] / 65536, (float)data[2] / 65536, accuracy);
      inv_get_sensor_type_compass(data, &accuracy, (inv_time_t *)&timestamp);
      serial.printf("Mx:%f My%f Mz%f Acc %d\r\n", (float)data[0] / 65536, (float)data[1] / 65536, (float)data[2] / 65536, accuracy);
    }
  }
}

