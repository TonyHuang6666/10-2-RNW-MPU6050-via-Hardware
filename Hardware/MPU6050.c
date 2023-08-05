#include "stm32f10x.h"
#include "MPU6050_Reg.h"

#define SlaveAddress 0xD0 //MPU6050的从机地址

//封装指定地址的写操作
void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    /*
    IIC_Start();
    IIC_SendByte(SlaveAddress);
    IIC_ReceiveACK();
    IIC_SendByte(reg);
    IIC_ReceiveACK();
    IIC_SendByte(data);
    IIC_ReceiveACK();
    IIC_Pause();
    */
    I2C_GenerateSTART(I2C2,ENABLE);//产生起始位
    I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Transmitter);//发送从机地址
    while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//等待从机应答
    I2C_SendData(I2C2,reg);//发送寄存器地址
    while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//等待数据发送完毕
    I2C_SendData(I2C2,data);//发送数据
    while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//等待数据发送完毕
    I2C_GenerateSTOP(I2C2,ENABLE);//产生停止位

}

//封装指定地址的读操作
uint8_t MPU6050_ReadReg(uint8_t reg)
{
    /*
    uint8_t data;
    //设置MPU6050当前地址指针
    IIC_Start();
    IIC_SendByte(SlaveAddress);
    IIC_ReceiveACK();
    IIC_SendByte(reg);
    IIC_ReceiveACK();
    //重新寻址
    IIC_Start();
    IIC_SendByte(SlaveAddress|0x01);//读操作
    //接收数据
    data = IIC_ReceiveByte();
    IIC_SendACK(1);//只读取一个字节，所以最后一个字节需要发送NACK
    IIC_Pause();
    return data;
    */
    uint8_t data;
    //设置MPU6050当前地址指针
    I2C_GenerateSTART(I2C2,ENABLE);//产生起始位
    I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Transmitter);//发送从机地址
    while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//等待从机应答
    I2C_SendData(I2C2,reg);//发送寄存器地址
    while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//等待数据发送完毕
    //重新寻址
    I2C_GenerateSTART(I2C2,ENABLE);//产生起始位
    I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Receiver);//发送从机地址,读操作
    while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//等待从机应答
    //接收数据
    data = I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2,DISABLE);//只读取一个字节，所以最后一个字节需要发送NACK
    I2C_GenerateSTOP(I2C2,ENABLE);//产生停止位
    return data;
}

void MPU6050_Init(void)
{

    //IIC初始化
    //1.GPIO初始化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//复用：引脚控制外设 开漏：IIC的时钟线是双向的
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    //2.IIC初始化
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);//是APB1的外设
    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//I2C模式
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//使能应答
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//7位地址
    I2C_InitStructure.I2C_ClockSpeed = 400000;//400KHz
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//占空比2
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;//主机地址
    I2C_Init(I2C2,&I2C_InitStructure);
    I2C_Cmd(I2C2, ENABLE);

    //1.配置电源管理器
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);//解除睡眠，选择时钟源为陀螺仪时钟
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);//不待机
    //2.配置采样率
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x07);
    //3.配置配置寄存器
    MPU6050_WriteReg(MPU6050_CONFIG,0x06);//配置最平滑的低通滤波器，带宽为5Hz
    //4.配置陀螺仪
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);//，前三位自测，不自测，配置量程为最大
    //5.配置加速度计
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x18);//不自测，配置量程为16g，最大
}

//方法1：用指针的方式返回六个返回值。要改进的话可以从0x3B开始连续读取14个寄存器的值
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
                        int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
    //1.读取加速度计X轴的数据
    uint8_t DataH= MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);//读取加速度计的X轴高八位
    uint8_t DataL= MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);//读取加速度计的X轴低八位
    *AccX = (DataH<<8)|DataL;//合成X轴的十六位数据。会自动进行数据转换，因为左边是int16_t类型
    //2.读取加速度计Y轴的数据
    DataH= MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);//读取加速度计的Y轴高八位
    DataL= MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);//读取加速度计的Y轴低八位
    *AccY = (DataH<<8)|DataL;//合成Y轴的十六位数据
    //3.读取加速度计Z轴的数据
    DataH= MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);//读取加速度计的Z轴高八位
    DataL= MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);//读取加速度计的Z轴低八位
    *AccZ = (DataH<<8)|DataL;//合成Z轴的十六位数据
    //4.读取陀螺仪X轴的数据
    DataH= MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);//读取陀螺仪的X轴高八位
    DataL= MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);//读取陀螺仪的X轴低八位
    *GyroX = (DataH<<8)|DataL;//合成X轴的十六位数据
    //5.读取陀螺仪Y轴的数据
    DataH= MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);//读取陀螺仪的Y轴高八位
    DataL= MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);//读取陀螺仪的Y轴低八位
    *GyroY = (DataH<<8)|DataL;//合成Y轴的十六位数据
    //6.读取陀螺仪Z轴的数据
    DataH= MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);//读取陀螺仪的Z轴高八位
    DataL= MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);//读取陀螺仪的Z轴低八位
    *GyroZ = (DataH<<8)|DataL;//合成Z轴的十六位数据
}

uint8_t MPU6050_ReadID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

//方法2：用结构体的方式返回六个返回值
/*
void MPU6050_GetData2(MPU6050_Data *data)
{
    //1.读取加速度计X轴的数据
    uint8_t DataH= MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);//读取加速度计的X轴高八位
    uint8_t DataL= MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);//读取加速度计的X轴低八位
    data->AccX = (DataH<<8)|DataL;//合成X轴的十六位数据。会自动进行数据转换，因为左边是int16_t类型
    //2.读取加速度计Y轴的数据
    DataH= MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);//读取加速度计的Y轴高八位
    DataL= MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);//读取加速度计的Y轴低八位
    data->AccY = (DataH<<8)|DataL;//合成Y轴的十六位数据
    //3.读取加速度计Z轴的数据
    DataH= MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);//读取加速度计的Z轴高八位
    DataL= MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);//读取加速度计的Z轴低八位
    data->AccZ = (DataH<<8)|DataL;//合成Z轴的十六位数据
    //4.读取陀螺仪X轴的数据
    DataH= MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);//读取陀螺仪的X轴高八位
    DataL= MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);//读取陀螺仪的X轴低八位
    data->GyroX = (DataH<<8)|DataL;//合成X轴的十六位数据
    //5.读取陀螺仪Y轴的数据
    DataH= MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);//读取陀螺仪的Y轴高八位
    DataL= MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);//读取陀螺仪的Y轴低八位
    data->GyroY = (DataH<<8)|DataL;//合成Y轴的十六位数据
    //6.读取陀螺仪Z轴的数据
    DataH= MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);//读取陀螺仪的Z轴高八位
    DataL= MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);//读取陀螺仪的Z轴低八位
    data->GyroZ = (DataH<<8)|DataL;//合成Z轴的十六位数据
}
*/