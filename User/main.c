#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "IIC.h"
#include "MPU6050.h"
int16_t AccX,AccY,AccZ,GyroX,GyroY,GyroZ;//定义存储加速度计和陀螺仪数据的变量
uint8_t ID;
int main(void)
{
	OLED_Init();
	MPU6050_Init();
	OLED_ShowString(4,16,"T");
	OLED_ShowString(4,1,"ID:");
	ID = MPU6050_ReadID();
	OLED_ShowHexNum(4,4,ID,2);
	while (1)
	{
		MPU6050_GetData(&AccX,&AccY,&AccZ,&GyroX,&GyroY,&GyroZ);
		OLED_ShowSignedNum(1,1,AccX,5);
		OLED_ShowSignedNum(2,1,AccY,5);
		OLED_ShowSignedNum(3,1,AccZ,5);
		OLED_ShowSignedNum(1,8,GyroX,5);
		OLED_ShowSignedNum(2,8,GyroY,5);
		OLED_ShowSignedNum(3,8,GyroZ,5);
	}
}
