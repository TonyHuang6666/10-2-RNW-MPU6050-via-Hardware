#ifndef IIC_H_
#define IIC_H_
//以下为IIC.c文件中的6个函数的声明
void IIC_Init(void);
void IIC_Start(void);
void IIC_Pause(void);
void IIC_SendByte(uint8_t Byte);
uint8_t IIC_ReceiveByte(void);
void IIC_SendACK(uint8_t ACK);
uint8_t IIC_ReceiveACK(void);
#endif