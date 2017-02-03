/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_can.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_adc.h"
#include "stm32f2xx_dma.h"
#include "stdbool.h"
#include "misc.h"

  CanTxMsg TxMessage;
  CanRxMsg RxMessage;
uint8_t KeyNumber = 0x0;

void CAN_Config(void);
void Init_RxMes(CanRxMsg *RxMessage);
void Delay(void);

  uint32_t uwCounter = 0;
  uint8_t TransmitMailbox = 0;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

__IO uint32_t ret = 0;
volatile TestStatus TestRx;
uint8_t txStat = 0;
uint8_t lastError = 0;
uint8_t rxStat = 0;
uint8_t previousState = 0;

#define ADC3_DR_ADDRESS    ((uint32_t)0x4001224C)
__IO uint16_t ADC3ConvertedValue = 0;
__IO uint32_t ADC3ConvertedVoltage = 0;

void ADC3_CH2_Config(void);

int main(void)
{
__enable_irq();

  /* CAN configuration */
  ADC3_CH2_Config();  
  /* Start ADC3 Software Conversion */ 
  ADC_SoftwareStartConv(ADC3);
  CAN_Config();
  
  /* Infinite loop */
  while(1)
  {
    // А тут мы ничего не делаем, вся работа у нас в прерывании
        __NOP();
        //TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
   }
}

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void CAN_Config(void)
{
   
  /* CAN_1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
   
  //Кофигурируем GPIO ножки PA11, PA12 для CAN1
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  //Tx
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  //Rx
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
  
  /* CAN1 reset */
  CAN_DeInit(CAN1);
  
  //Конфигурируем CAN1
  CAN_InitTypeDef       CAN_InitStructure;
    
  CAN_InitStructure.CAN_TTCM = DISABLE; // time-triggered communication mode = DISABLED
  CAN_InitStructure.CAN_ABOM = DISABLE; // automatic bus-off management mode = DISABLED
  CAN_InitStructure.CAN_AWUM = DISABLE; // automatic wake-up mode = DISABLED
  CAN_InitStructure.CAN_NART = DISABLE; // non-automatic retransmission mode = DISABLED
  CAN_InitStructure.CAN_RFLM = DISABLE; // receive FIFO locked mode = DISABLED
  CAN_InitStructure.CAN_TXFP = DISABLE; // transmit FIFO priority = DISABLED
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //CAN_Mode_LoopBack; //CAN_Mode_Normal; //CAN_Mode_Silent_LoopBack; //
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; // synchronization jump width = 1
  
  /* CAN Baudrate = 1000kbps (CAN clocked at 30 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 2;
  CAN_Init(CAN1, &CAN_InitStructure);

   //Конфигурируем фильтр сообщений CAN`a
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
    
  CAN_FilterInitStructure.CAN_FilterNumber = 0; // filter number = 0 (0<=x<=13)
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;  //ENABLE
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  //Кофигурируем NVIC прерывания
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  
    /* Transmit Structure preparation */
    TxMessage.StdId = 0x77;
    TxMessage.ExtId = 0x00;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0x00;
    TxMessage.Data[1] = 0x00;
  
}

void CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;

  RxMessage.StdId = 0x00;
  RxMessage.ExtId = 0x00;
  RxMessage.IDE = 0;
  RxMessage.DLC = 0;
  RxMessage.FMI = 0;
  for (uint8_t DLC_i = 0; DLC_i < 7; DLC_i ++)
  {
      RxMessage.Data[DLC_i] = 0x00;
  }
  
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

  if((RxMessage.StdId == 0x12) && (RxMessage.DLC == 5)
     && (RxMessage.Data[0] == 'S')
     && (RxMessage.Data[1] == 'T')
     && (RxMessage.Data[2] == 'A')
     && (RxMessage.Data[3] == 'R')
     && (RxMessage.Data[4] == 'T'))
  {
    ADC3ConvertedVoltage = ADC3ConvertedValue *3300/0xFFF;
    
    /* Transmit Structure preparation */
    TxMessage.StdId = 0x77;
    TxMessage.ExtId = 0x00;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = (uint8_t) (ADC3ConvertedVoltage >> 8);
    TxMessage.Data[1] = (uint8_t) (ADC3ConvertedVoltage);
    TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
    /*
    uwCounter = 0;
    while((CAN_TransmitStatus(CAN1, TransmitMailbox)  !=  CANTXOK) && (uwCounter  !=  0xFFFF))
    {
      uwCounter++;
    }
    uwCounter = 0;
    while((CAN_MessagePending(CAN1, CAN_FIFO0) < 1) && (uwCounter  !=  0xFFFF))
    {
      uwCounter++;
    }
    */
  }
}

void ADC3_CH2_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel2 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
