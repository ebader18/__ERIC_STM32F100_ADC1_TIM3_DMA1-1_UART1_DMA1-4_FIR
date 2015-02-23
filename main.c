#include "main.h"
#include "stm32f10x.h"
#include <string.h>

#define ADC1_DR_Address                 ((uint32_t)0x4001244C)
#define USART1_DR_Base                  ((uint32_t)0x40013804)

void Configure_Data();
void Configure_RCC();
void Configure_GPIOs();
void Configure_NVIC();
void Configure_DMA1_1();
void Configure_DMA1_4(uint32_t pTXData);
void Configure_ADC1();
void Configure_TIM3();
void Configure_UART1();

volatile short ADCValues[2048], lastADCValues[32 - 1];
volatile short TXData[1024];

int main(void)
{
  Configure_Data();
  Configure_RCC();
  Configure_GPIOs();
  Configure_NVIC();
  Configure_DMA1_1();
  Configure_ADC1();
  Configure_TIM3();
  Configure_UART1();
  
  __CLEAR_BIT(GPIOC->ODR, 8);
  __CLEAR_BIT(GPIOC->ODR, 9);
  
  DMA_Cmd(DMA1_Channel1, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  TIM_Cmd(TIM3,ENABLE);
  
  USART_Cmd(USART1, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  
  while(1)
  {
  }
}

void Configure_Data()
{
// http://t-filter.appspot.com/fir/index.html //
  /*sampling frequency: 16000 Hz, fixed point precision: 16 bits
* 0 Hz - 450 Hz, gain = 0, desired attenuation = -40 dB
* 500 Hz - 2500 Hz, gain = 1, desired ripple = 5 dB
* 2600 Hz - 8000 Hz, gain = 0, desired attenuation = -40 dB*/
  //short tap[32] = {2240,2319,2702,2536,1811,556,-1253,-3626,-6318,-8603,-9356,-7520,-2812,3814,10313,14339,14339,10313,3814,-2812,-7520,-9356,-8603,-6318,-3626,-1253,556,1811,2536,2702,2319,2240};
  
  /*sampling frequency: 16000 Hz, fixed point precision: 16 bits
* 0 Hz - 1000 Hz, gain = 0, desired attenuation = -40 dB
* 1100 Hz - 1800 Hz, gain = 1, desired ripple = 1 dB
* 2000 Hz - 8000 Hz, gain = 0, desired attenuation = -40 dB*/
  //short tap[32] = {-5197,-881,412,2283,4000,4783,4082,1907,-1131,-3979,-5542,-5122,-2738,838,4338,6485,6485,4338,838,-2738,-5122,-5542,-3979,-1131,1907,4082,4783,4000,2283,412,-881,-5197};
  
  for(uint16_t idxSample = 0; idxSample < 31; idxSample++)
    lastADCValues[idxSample] = 0;
}

void ADC_ready(uint16_t idxDMAcurrentPos)
{
  //uint16_t c1, c2;
  uint16_t idxDataTx, idxSample, idxTap;
  int32_t tSample = 0;
  short tap[32] = {-5197,-881,412,2283,4000,4783,4082,1907,-1131,-3979,-5542,-5122,-2738,838,4338,6485,6485,4338,838,-2738,-5122,-5542,-3979,-1131,1907,4082,4783,4000,2283,412,-881,-5197};
  
  if (idxDMAcurrentPos == 1024)
    idxDataTx = 1024;
  else if (idxDMAcurrentPos == 2048)
    idxDataTx = 0;
  
  //c1 = DMA_GetCurrDataCounter(DMA1_Channel1);
  for(idxSample = 0; idxSample < 32 - 1; idxSample++)
  {
    tSample = tap[0] * ADCValues[idxSample];
    for(idxTap = 0; idxTap < idxSample; idxTap++)
      tSample += tap[idxTap + 1] * ADCValues[idxDataTx + idxSample - idxTap - 1];
    for(idxTap = 1; idxTap < 32 - idxSample; idxTap++)
      tSample += tap[idxSample + idxTap] * lastADCValues[idxTap - 1];
    TXData[idxSample] = (short)(tSample>>16);
  }
  for(idxSample = 32 - 1; idxSample < 1024-1; idxSample++)
  {
    tSample = tap[0] * ADCValues[idxSample];
    for(idxTap = 1; idxTap < 32; idxTap++)
      tSample += tap[idxTap] * ADCValues[idxDataTx + idxSample - idxTap];
    TXData[idxSample] = (short)(tSample>>16);
  }
  for(idxSample = 0; idxSample < 31; idxSample++)
    lastADCValues[31 - idxSample - 1] = ADCValues[idxDataTx + idxSample + 1024 - 32 + 1];
  //c2 = DMA_GetCurrDataCounter(DMA1_Channel1);
  
  //Configure_DMA1_4((uint32_t)&ADCValues[idxDataTx]);
  Configure_DMA1_4((uint32_t)&TXData[0]);
  DMA_Cmd(DMA1_Channel4, ENABLE);
}

void Configure_RCC()
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                                           // Enable GPIOC Clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                                           // Enable ADC1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                                           // Enable GPIOA Clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                                           // Enable Alternate Fuctions Clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                                           // Enable TIM3 Clock
  RCC->AHBENR |= RCC_AHBPeriph_DMA1;                                            // Enable DMA1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                                         // Enable UART1 Clock
}

void Configure_GPIOs()
{
  GPIOC->CRH = 0x44444422;                                                      // PC8 and PC9 as General Purpose Push Pull Output Max Speed = 2MHz
  GPIOA->CRL = 0x44444404;                                                      // PA1 as input analog mode
  GPIOA->CRH = 0x22224444;                                                      // PA12, PA13, PA14 & PA15 as General Purpose Push Pull Output Max Speed = 2MHz
  
  __CLEAR_BIT(GPIOC->ODR, 8);
  __CLEAR_BIT(GPIOC->ODR, 9);
  __CLEAR_BIT(GPIOA->ODR, 12);
  __CLEAR_BIT(GPIOA->ODR, 13);
  __CLEAR_BIT(GPIOA->ODR, 14);
  __CLEAR_BIT(GPIOA->ODR, 15);
}

void Configure_NVIC()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;                      // Enable DMA1 channel1 IRQ Channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;                      // Enable DMA1 channel4 IRQ Channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Configure_DMA1_1()
{
  DMA_InitTypeDef DMA_InitStructure;
  
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCValues[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2048;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
}

void Configure_DMA1_4(uint32_t pTXData)
{
  DMA_InitTypeDef DMA_InitStructure;
  
  DMA_DeInit(DMA1_Channel4);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = pTXData;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 2048;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
}

void Configure_ADC1()
{
  ADC_InitTypeDef ADC_InitStructure;
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
}

void Configure_TIM3()
{
  TIM_TimeBaseInitTypeDef TimeBaseInitStructure; 
  
  TimeBaseInitStructure.TIM_Prescaler = 24 - 1;
  TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TimeBaseInitStructure.TIM_Period = 1000/16;                                        // in microseconds
  TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  
  TIM_TimeBaseInit(TIM3,&TimeBaseInitStructure);
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
}

void Configure_UART1()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 921600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure);
}