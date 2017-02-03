/**
  ******************************************************************************
  * @file    app.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides all the Application firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/ 

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

  
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
   
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

//Переменные USB
extern volatile uint8_t APP_Rx_Buffer [];
extern volatile uint32_t APP_Rx_ptr_in;
//Переменные TIM (для отладки)
uint8_t previousState=0;
//Переменные CAN
CanTxMsg TxMessage;
CanRxMsg RxMessage;

uint8_t mybyte = 0;
uint8_t i = 0;


struct Packet
  {
    uint8_t START;
    uint8_t LENP;
    uint8_t POT[4];
    uint8_t CRCs;
  };
struct Packet DataPacket;

void USB_CDC_buffer_write_byte(uint8_t byte);
void SendPacket(void);
void ResetPacket(void);
void TIM4_Config(void);
void CAN_Config(void);

int main(void)
{
   

  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32fxxx_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32fxxx.c file
  */  
 
  USBD_Init(&USB_OTG_dev,     
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
   
  ResetPacket();
  
  TIM4_Config();
  CAN_Config();
  
  /* Main loop */
  while (1)
  {

  }
} 

void USB_CDC_buffer_write_byte(uint8_t byte) {
  APP_Rx_Buffer[APP_Rx_ptr_in++] = byte;
  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
  {
    APP_Rx_ptr_in = 0;
  }
}

void SendPacket(void)
{
    USB_CDC_buffer_write_byte(DataPacket.START++);
    USB_CDC_buffer_write_byte(DataPacket.LENP);
    for (uint8_t counter = 0; counter < (DataPacket.POT[1]+2); counter ++ )
    {
      USB_CDC_buffer_write_byte(DataPacket.POT[counter]);
    }
    USB_CDC_buffer_write_byte(DataPacket.CRCs);
}

void ResetPacket(void)
{
    DataPacket.START = 0x00;
    DataPacket.LENP = 7;
    DataPacket.POT[0] = 0x77;
    DataPacket.POT[1] = 2;
    DataPacket.POT[2] = 0;
    DataPacket.POT[3] = 0;
    DataPacket.CRCs = 0xFF;
}

void CAN_Config(void);

void TIM4_Config(void)
{
    TIM_TimeBaseInitTypeDef timer;
    //Включаем тактирование таймера TIM4
    //Таймер 4 у нас висит на шине APB1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //Заполняем поля структуры дефолтными значениями
    TIM_TimeBaseStructInit(&timer);
    //Выставляем предделитель
    timer.TIM_Prescaler = 300-1;
    //Тут значение, досчитав до которого таймер сгенерирует прерывание
    //Кстати это значение мы будем менять в самом прерывании
    timer.TIM_Period = 2000-1; //Частота 100Гц
    //Инициализируем TIM4 нашими значениями
    TIM_TimeBaseInit(TIM4, &timer);	
    //Настраиваем таймер для генерации прерывания по обновлению (переполнению)
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //Запускаем таймер 
    TIM_Cmd(TIM4, ENABLE);
    //Разрешаем соответствующее прерывание
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
   
}

void TIM4_IRQHandler()
{	
     TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    //Если на выходе был 0..
    if (previousState == 0)
    {
	//Выставляем единицу на выходе
	previousState = 1;
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
    else
    {
	//Выставляем ноль на выходе
	previousState = 0;
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);	
    }
    
    SendPacket();
    
    CAN_Transmit(CAN1, &TxMessage);
}

void CAN_Config(void)
{   
    /* CAN_1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
    //Кофигурируем GPIO ножки PB9, PB8 для CAN1
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    //Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
  
    /* CAN1 reset */
    CAN_DeInit(CAN1);
    
    //Конфигурируем CAN1
    CAN_InitTypeDef       CAN_InitStructure;
    
  CAN_InitStructure.CAN_TTCM = DISABLE; // time-triggered communication mode = DISABLED
  CAN_InitStructure.CAN_ABOM = DISABLE; // automatic bus-off management mode = DISABLED
  CAN_InitStructure.CAN_AWUM = DISABLE; // automatic wake-up mode = DISABLED
  CAN_InitStructure.CAN_NART = ENABLE; // non-automatic retransmission mode = DISABLED
  CAN_InitStructure.CAN_RFLM = DISABLE; // receive FIFO locked mode = DISABLED
  CAN_InitStructure.CAN_TXFP = DISABLE; // transmit FIFO priority = DISABLED
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //CAN_Mode_LoopBack; //CAN_Mode_Normal; //CAN_Mode_Silent_LoopBack; //
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; // synchronization jump width = 1
   
  /* CAN Baudrate = 1MBps (CAN clocked at 30 MHz) */ /* CAN Baudrate = 175kbps (CAN clocked at 42 MHz) */
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq; //CAN_BS1_6tq
    CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq; //CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = 2; //16
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
/*
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  */
    
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
 //   CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);    

    /* Transmit Structure preparation */
    TxMessage.StdId = 0x12;
    TxMessage.ExtId = 0x00;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = 5;
    TxMessage.Data[0] = 'S';
    TxMessage.Data[1] = 'T';
    TxMessage.Data[2] = 'A';
    TxMessage.Data[3] = 'R';
    TxMessage.Data[4] = 'T';
}

void CAN1_RX0_IRQHandler(void)
{
  if (CAN_GetITStatus(CAN1,CAN_IT_FMP0))
  {
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    
    RxMessage.StdId = 0x00;
    RxMessage.IDE = CAN_ID_STD;
    RxMessage.DLC = 0;
    for (uint8_t DLC_i = 0; DLC_i < 8; DLC_i ++)
      {
        RxMessage.Data[DLC_i] = 0x00;
      }
  
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    CAN_FIFORelease(CAN1,CAN_FIFO0);
    
    if ((RxMessage.StdId == 0x77))
  {
    DataPacket.POT[0] = RxMessage.StdId;
    DataPacket.POT[1] = RxMessage.DLC;
    for (uint8_t counter = 0; counter < RxMessage.DLC; counter ++ )
    {
      DataPacket.POT[counter+2] = RxMessage.Data[counter];
    }
  }
  }
}
 



#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
