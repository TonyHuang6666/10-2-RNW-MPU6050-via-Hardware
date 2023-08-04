#include "stm32f10x.h"
#include "Delay.h"
#define Delay_Time 10
//对于主频高的单片机，不建议使用宏定义

void IIC_Init(void)
{
    //1.SCL和SDA线开漏输出模式
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //2.SCL和SDA都置高电平
    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);//SCL和SDA都置高电平,IIC总线属于空闲状态
}

//以下四个函数为对引脚操作的封装和改名，方便移植

void IIC_W_SCL(uint8_t BitValue)//写SCL引脚电平
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
    Delay_us(Delay_Time);
    //对于f103系列，即使不加延时函数引脚的电平翻转速度MPU6050也跟得上
}

void IIC_W_SDA(uint8_t BitValue)//写SDA引脚电平
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
    Delay_us(Delay_Time);
}

uint8_t IIC_R_SCL(void)//读SCL引脚电平
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10);
    Delay_us(Delay_Time);
    return BitValue;
}

uint8_t IIC_R_SDA(void)//读SDA引脚电平
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
    Delay_us(Delay_Time);
    return BitValue;
}

//以下为IIC的六种基本条件

void IIC_Start(void)//1.IIC起始条件
{
    //1.SCL和SDA都置高电平,IIC总线空闲状态（释放）
    IIC_W_SDA(1);//放前面保险一点，要兼容重复起始条件Sr.Sr最开始SCL是低电平，但SDA不确定，不一定是xx沿
    IIC_W_SCL(1);
    //2.起始条件:SCL高电平期间，SDA下降沿，SCL紧随
    IIC_W_SDA(0);
    IIC_W_SCL(0);
}

void IIC_Pause(void)//2.IIC终止条件
{
    //1.SCL和SDA都置低电平
    IIC_W_SDA(0);//先拉低SDA，确保后面能产生上升沿
    //2.终止条件:SCL高电平期间，SDA上升沿
    IIC_W_SCL(1);//除了终止条件，SCL都是以低电平结束
    IIC_W_SDA(1);
}

void IIC_SendByte(uint8_t Byte)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
        IIC_W_SDA(Byte & 0x80);//写入数据的最高位
        Byte<<=1;//数据左移一位
        IIC_W_SCL(1);//释放SCL
        IIC_W_SCL(0);//拉低SCL
    }
}

uint8_t IIC_ReceiveByte(void)
{
    uint8_t i,Byte=0x00;
    IIC_W_SDA(1);//为了避免主机干扰从机写入数据，先释放SDA(也相当于切为输入模式)
    for(i=0;i<8;i++)
    {
        IIC_W_SCL(1);//释放SCL
        Byte|=IIC_R_SDA();//读取数据的最高位。|= 是按位或赋值运算符，它将左和右按位或后再赋值给左
        IIC_W_SCL(0);//拉低SCL
        Byte<<=1;//数据左移一位
    }
    return Byte;
}

void IIC_SendACK(uint8_t ACK)//3.IIC发送应答
{
    IIC_W_SDA(ACK);//写入应答位
    IIC_W_SCL(1);//释放SCL
    IIC_W_SCL(0);//拉低SCL
}

uint8_t IIC_ReceiveACK(void)//4.IIC接收应答
{
    uint8_t ACK;
    IIC_W_SDA(1);//释放SDA
    IIC_W_SCL(1);//释放SCL
    ACK=IIC_R_SDA();//读取应答位
    IIC_W_SCL(0);//拉低SCL
    return ACK;
}

