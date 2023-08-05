#include "stm32f10x.h"
#include "MPU6050_Reg.h"

#define SlaveAddress 0xD0 //MPU6050的从机地址

void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)//给CheckEvent函数封装一个延时函数，防止死机
{
    uint32_t timeout=10000;
    while(I2C_CheckEvent(I2Cx, I2C_EVENT)!=SUCCESS)
    {
        timeout--;
        if(timeout==0)
        {
            break;//超时退出。这里可以加入其他处理，比如重新初始化I2C等等升级操作及处理
        }
    }
}

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
    //上面软件的函数都有Delay，是阻塞式的，下面的函数是硬件的、非阻塞式的，要等待响应标志位，确保执行完毕
    I2C_GenerateSTART(I2C2,ENABLE);//产生起始位
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);//等待起始位发送完毕,即EV5
    I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Transmitter);//发送从机地址。自带应答。
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);//等待从机应答,即EV6
    I2C_SendData(I2C2,reg);//发送寄存器地址
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING);//等待数据发送完毕,即EV8
    I2C_SendData(I2C2,data);//发送数据
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);//发送多字节时，等待EV8_2,最后一个字节发送完毕
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
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);//等待起始位发送完毕,即EV5
    I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Transmitter);//发送从机地址。自带应答。
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);//等待从机应答,即EV6
    I2C_SendData(I2C2,reg);//发送寄存器地址
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);//等待最后一个字节发送完毕,即EV8_2
    //重新寻址
    I2C_GenerateSTART(I2C2,ENABLE);//产生起始位
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);//等待起始位发送完毕,即EV5
    I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Receiver);//发送从机地址,读操作
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);//等待主机接受,即EV6。与上面的EV6不同
    

    //如果要接收多个字节，则以下四行代码要放在循环中，前面只执行最后两行，计数到了最后一个字节则四行都执行
    //对于最后一个字节，要提前应答位ACK置0，停止条件生成位stop置1
    I2C_AcknowledgeConfig(I2C2,DISABLE);//只读取一个字节，所以最后一个字节需要发送NACK
    I2C_GenerateSTOP(I2C2,ENABLE);//产生停止位
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED);//等待数据接收完毕,即EV7
    //接收数据
    data = I2C_ReceiveData(I2C2);

    I2C_AcknowledgeConfig(I2C2,ENABLE);//恢复应答位ACK
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
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//stm32作为从机模式时7位地址
    I2C_InitStructure.I2C_ClockSpeed = 50000;//50KHz
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//低于100KHz的标准速度此参数无意义，高速时要增大低电平占比
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;//stm32作为从机模式时的自身地址
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