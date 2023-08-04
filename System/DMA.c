#include "stm32f10x.h"
uint16_t AD_Value[4];
void DMA_Initilize(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//外设地址为ADC1的DR寄存器
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//ADC1->DR是16位的，所以这里是半字
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不变
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;//存放ADC值的数组
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存放ADC值的数组是16位的，所以这里是半字
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存放ADC值的数组递增
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设作为数据源
    DMA_InitStructure.DMA_BufferSize = 4;//DMA缓存大小为4
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // 不是软件触发，而是ADC1触发
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//DMA优先级中等
    DMA_Init(DMA1_Channel1, &DMA_InitStructure); // 硬件触发，通道不可以任意选择
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE); // ADC触发DMA
}