#include "stm32f10x.h"
#include "stdarg.h"
#include "stdio.h"

//面对数据可能重叠的问题，可以再定义一个数组
char Serial_RxPackage[100];
uint8_t Serial_RxFlag;

void Serial_Initilize(void)
{
	//1.配置时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//2.配置GPIO。TX用复用推挽输出；RX用浮空输入或者上拉输入，因为串口空闲状态为高电平
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//3.配置串口
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	//4.开启串口中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启RXNE标志位到NVIC的输出
	//5.配置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断分组
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//选择串口1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能中断
	NVIC_Init(&NVIC_InitStructure);
	//6.开启串口
	USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t data)
{
    USART_SendData(USART1, data);//data变量写入到USART1_DR寄存器中
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//等待，直到发送完成，即TXE=1，TXE标志位置SET
    //标志位不需要手动清零，写入新数据时会自动清零
}

void Serial_SendString(uint8_t *str)
{
	while(*str != '\0')
	{
		Serial_SendByte(*str);
		str++;
	}
}

void Serial_SendArray(uint8_t *array, uint16_t length)
{
	uint16_t i;
	for(i = 0; i < length; i++)
	{
		Serial_SendByte(array[i]);
	}
}
/*
作用是计算一个数的幂。它接收两个参数，一个是底数 base，一个是指数 exponent。
函数内部使用一个循环来计算幂，循环次数为指数 exponent。在每次循环中，将 result 乘以底数 base，最后返回 result 的值。
*/
uint32_t Power(uint8_t base, uint8_t exponent)
{
	uint32_t result = 1;
	uint8_t i;
	for(i = 0; i < exponent; i++)
	{
		result *= base;
	}
	return result;
}
/*
作用是将一个无符号整数 Number 以十进制形式发送出去。它接收两个参数，一个是要发送的数字 Number，一个是数字
的位数 Length。函数内部使用一个循环来将数字的每一位发送出去。在每次循环中，先计算出当前位的数字，然后
将其转换为 ASCII 码并发送出去。具体的转换方法是将当前位的数字除以 10 的 Length - i - 1 次方，
然后对 10 取模，再加上字符 '0'
*/
void Serial_SendNumber(uint32_t Number,uint8_t Length)
{
	uint8_t i;
	for(i = 0; i < Length; i++)
	{
		Serial_SendByte((Number / Power(10,Length - i - 1)) % 10 + '0');
	}
}

//以下函数用于重定向printf函数,printf函数就是调用fputc函数一个个打印的
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return (ch);
}

// 这个函数的作用是将格式化的字符串通过串口发送出去
void Serial_Printf(char *format,...)
{
	// 定义一个字符数组，用于存储格式化后的字符串
	char String[100];
	// 定义一个指向参数列表的指针
	va_list arg;
	// 初始化指针，指向可变参数列表
	va_start(arg,format);
	// 将可变参数列表中的参数按照指定格式格式化到字符数组中
	vsprintf(String,format,arg);
	// 结束可变参数列表的访问
	va_end(arg);
	// 将格式化后的字符串通过串口发送出去
	Serial_SendString(String);
}

void USART1_IRQHandler(void)//状态机思想
{
	static uint8_t RxState = 0;//0:等待包头 1:接收数据 2:接收完毕
	static uint8_t Queue = 0;//接收队列
	//1.判断标志位
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);
		//if+else if比起if+if有优先级功能，只会执行一个，不会同时满足
		if (RxState == 0)
		{
			//Serial_RxFlag这个值需要在状态0里置0，否则只要一次为1，永远为1。
			//Serial_RxFlag = 0;
			if (RxData == '@')//如果发送太快，则跳过该数据包，所以加上Serial_RxFlag==0
													//即上一包接收完毕才能接收下一包，避免数据丢失，再或者就定义缓存
			{
				RxState = 1;
				Queue = 0;
			}
		}//等待包头
		else if (RxState == 1)
		{
			if (RxData == '\r')
			{
				RxState = 2;
			}
			else
			{
				Serial_RxPackage[Queue] = RxData;
				Queue ++;
			}
		}//接收数据
		else if (RxState == 2)
		{
			if (RxData == '\n')
			{
				RxState=0;//接收完毕，等待下一包
				Serial_RxPackage[Queue]='\0';//字符串结尾
				Serial_RxFlag=1;//置位标志位
			}
		}//等待包尾
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

uint8_t Serial_GetRxFlag(void)
{
	if(Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}
//经过测试，这里有个bug的，输入一段字符之后不换行，点发送，然后再随便添加几个字符，这次加换行，点发送，显示会错误的
//因为他换行没删啊，他只是把第一行字换了，你如果鼠标点第二行还是能发现一个空白行