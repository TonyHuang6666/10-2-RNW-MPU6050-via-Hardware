#ifndef MPU6050_H_
#define MPU6050_H_
/*
	以下三行相当于一个点名的时序，可以结合for循环遍历所有从机地址，
	即前7位，把应答位为0的地址打印出来，实现扫描总线上设备的功能
	IIC_Start();
	IIC_SendByte(0xD2);//1101001 0
	uint8_t ack = IIC_ReceiveACK();
*/
uint8_t MPU6050_ReadReg(uint8_t reg);
void MPU6050_WriteReg(uint8_t reg, uint8_t data);
void MPU6050_Init(void);
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
                        int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ);
uint8_t MPU6050_ReadID(void);
#endif