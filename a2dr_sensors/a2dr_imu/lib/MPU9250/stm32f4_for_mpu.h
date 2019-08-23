/*
 * mpu9250.h
 *
 *  Created on: Feb 4, 2019
 *      Author: PRUEK
 */

#ifndef MPU9250_STM32F4_FOR_MPU_H_
#define MPU9250_STM32F4_FOR_MPU_H_

#include "stdio.h"
#include <stdlib.h>
#include <string.h>
// #include "main.h"
// #include "i2c.h"


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"


struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
#ifdef __cplusplus
	extern "C" {
#endif

uint8_t Sensors_I2C_ReadRegister (uint8_t DevAddress, uint8_t RegAddress, uint8_t len, uint8_t *pData);
uint8_t Sensors_I2C_WriteRegister (uint8_t DevAddress, uint8_t RegAddress, uint8_t len, uint8_t *pData);
int get_tick_count (uint32_t *count);
void mdelay(unsigned long num_ms);
void run_self_test(void);
void MPU9250_Initial ();
int8_t MPU9250_Process ();
#ifdef __cplusplus
	}
#endif

#endif /* MPU9250_STM32F4_FOR_MPU_H_ */
