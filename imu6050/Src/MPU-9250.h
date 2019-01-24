#ifndef MPU
#define MPU

#include "stm32f4xx_hal.h"
extern volatile float gyro_x,gyro_y,gyro_z,accel_x=0,accel_y=0,accel_z=0,temp,mag_x, mag_y ,mag_z,angle =0;
extern long int gyro_cal_y;
extern float asax , asay ,asaz; //Initialing sensitivity adjustment values
extern	float Xa,Ya,Za,t;
extern	float Xg=0,Yg=0,Zg=0;
extern	double tot;
extern 	float angle1;
extern	float angle_gyro=0,del_x;
extern	int set_gyro_angle=0;

void MPU_Init(uint16_t slave_address);
void MPU_GET_VALUE(uint16_t slave_address);
void MPU_GYRO_CAL();
void MPU_SHOWDATA(void);
//Magnetometer
void BY_PASS_MPU(uint8_t slave_address );
void Init_Magnetometer(uint8_t MAG_AD);
void continuous_mode(uint8_t MAG_AD);
void Mag_get_value(uint8_t MAG_AD);

#endif