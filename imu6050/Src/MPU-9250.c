#include "MPU-9250.h"

void MPU_Init(uint16_t slave_address)
{
	char string[30] ;
	uint8_t buffer[2]={0x19,0x07};//Gyroscope output value is 8MHZ so sample rate is 1MHz
	
  while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 1st\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	
	buffer[0] = 0x6B;//Power Managment system
	buffer[1] = 0x00;//Internal clock of 8MHz oscillator
 while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 2nd");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//Configure
	buffer[0] = 0x1A;
	buffer[1] = 0x00;//Digital low pass filter Gyro = 8kHz
 while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 3rd \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}

	//Configure Gyro
	buffer[0] = 0x1B;
	buffer[1] = 0x00; // 
while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 4rd \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//Configure Accelerometer 
	buffer[0] = 0x1C;
	buffer[1] = 0x00;	
	while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 5rd \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	//Interrupt Enable
	buffer[0] = 0x38;
	buffer[1] = 0x01;
while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 6rd \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	
	
}

void MPU_GET_VALUE(uint16_t slave_address)
{
	uint8_t buffer=0x43;
	int8_t data[6];
	HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,1,200);
	//HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,slave_address<<1,(uint8_t *)&data,6,200);
//	accel_x = ~((((int)data[0]<<8|((int)data[1])))-1);//2`s complements value to signed value value
//	accel_y = ~((((int)data[2]<<8|((int)data[3])))-1);
//	accel_z =~((((int)data[4]<<8|((int)data[5])))-1);
//	temp = ((data[6]<<8)|(data[7]));
	gyro_x = ~((((int)data[0]<<8|((int)data[1])))-1);
	gyro_y = ~((((int)data[2]<<8|((int)data[3])))-1);
	gyro_y = gyro_y - gyro_cal_y;
	gyro_z = ~((((int)data[4]<<8|((int)data[5])))-1);
		

}
void MPU_GYRO_CAL()
{
	for(int i=0; i<200;i++)
	{
		MPU_GET_VALUE(0X68);
		gyro_cal_y += gyro_y;
	
	}
	gyro_cal_y /=200;
	
}

void MPU_SHOWDATA(void)
{
     
     MPU_GET_VALUE(0x68);
//     Xa = accel_x/16384;								/* Divide raw value by sensitivity scale factor to get real values */
//     Ya = accel_y/16384;
//     Za = accel_z/16384;
     
     Xg = gyro_x/131;
     Yg = gyro_y/131;
     Zg = gyro_z/131;
     
	  
//		sprintf(string,"%f \n",Xg);
//		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
//		sprintf(string,"%f \n",Yg);
//		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
//		sprintf(string,"Gyro_Z=%f ",Zg);
//		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
//		sprintf(string,"\n");
//		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
//		
	
//		 sprintf(string,"T= %d\n",TIM2->CNT);
// 		 HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
//		 
     
     del_x = (double)(TIM2->CNT) / 10000 ;
		 if (Yg<1 & Yg>-1) Yg =0;
     angle_gyro += Yg*del_x;	
		 TIM2->CNT = 0;	
		 sprintf(string,"angle_Yg = %f\t",Yg);
 		 HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
		 
		 sprintf(string,"angle = %f\n",angle_gyro);
 		 HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
		 
     //tot = sqrt(Xa*Xa+Ya*Ya+Za*Za);
     //angle1 = asin((float)Xa/tot)*57.2957795130;
	/*
     if(set_gyro_angle)
     {
	     angle_gyro = angle_gyro*0.96 +angle1*0.04;
     }
     else
     {
	     angle_gyro = angle1;
	     set_gyro_angle =1;
     }   
		 */

}
//For using Magnetometer I2C master interface must be disable and Bypass multiplexer must be enable
void BY_PASS_MPU(uint16_t slave_address )
{
	uint8_t buffer[2];
	//Disable the I2C Master interface and enable Bypass 
	buffer[0] = 0x6A;
	buffer[1] = 0x00;
 while(HAL_I2C_Master_Transmit(&hi2c1, slave_address<<1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Diable I2C Master interfaec \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//enable Bypass
	buffer[0] = 0x37;
	buffer[1] = 0x02;
 while(HAL_I2C_Master_Transmit(&hi2c1, slave_address<<1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Enable Bypass multiplexer \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	sprintf(string,"I have completed bypass code \n");
	HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	
}

//To get the data of sensitivity adjsutment value it must be in Fuse ROM access mode
void Init_Magnetometer(uint8_t MAG_AD)
{
	uint8_t buffer[2];
	uint8_t buff=0x10 ;//address of sensitivity adjustment value
	uint8_t data[3];
	buffer[0] = 0x0A ; 
	buffer[1] = 0x1F;//fuse mode access mode
	
 while(HAL_I2C_Master_Transmit(&hi2c1, MAG_AD << 1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Diable I2C Master interfaec \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	HAL_I2C_Master_Transmit(&hi2c1,MAG_AD<<1,(uint8_t *)&buff,1,200);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,MAG_AD<<1,(uint8_t *)&data,3,200);
	asax = (data[0]-128)*0.5/128+1;//Sensitivity adjustment value for x
	asay = (data[1]-128)*0.5/128+1;
	asaz = (data[2]-128)*0.5/128+1;
}
void continuous_mode(uint8_t MAG_AD)
{
	uint8_t buffer[2];
	buffer[0] = 0x0A ; 
	buffer[1] = 0x00;//power down mode
	
 while(HAL_I2C_Master_Transmit(&hi2c1, MAG_AD << 1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Diable I2C Master interfaec \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//Start the Magnetometer to Continuous mode 2 (100HZ) and 16 bit output
	
	buffer[0] = 0x0A ; 
	buffer[1] = 0x16; //0001 0110 in binary  continous mode
	
 while(HAL_I2C_Master_Transmit(&hi2c1, MAG_AD << 1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Continuous_mode \n");
		HAL_UART_Transmit(&huart1,(uint8_t *)&string,sizeof(string),0xFFFF);
	}

}
