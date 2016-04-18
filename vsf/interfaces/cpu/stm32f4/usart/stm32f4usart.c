#include <Stdint.h>
#include "stm32f4xx.h"
#include "vsf_err.h"
#include "App_type.h"
#include "interfaces.h"
#define STM32F4_AFIO_MAPR_USART1			((uint32_t)1 << 2)
#define STM32F4_AFIO_MAPR_USART2			((uint32_t)1 << 3)
#define STM32F4_AFIO_MAPR_USART3_SFT		((uint32_t)1 << 4)
#define STM32F4_AFIO_MAPR_USART3_MSK		((uint32_t)0X03 << \
																				STM32_AFIO_MAPR_USART3_SFT)
#define STM32F4_AFIO_MAPR_SWJCFG			((uint32_t)7 << 24)

#define STM32F4_RCC_APB1ENR_USART2EN		((uint32_t)1 << 17)
#define STM32F4_RCC_APB1ENR_USART3EN		((uint32_t)1 << 18)
#define STM32F4_RCC_APB1ENR_USART4EN		((uint32_t)1 << 19)
#define STM32F4_RCC_APB1ENR_USART5EN		((uint32_t)1 << 20)
#define STM32F4_RCC_APB2ENR_USART1EN		((uint32_t)1 << 14)
#define STM32F4_RCC_APB2ENR_IOPAEN		((uint32_t)1 << 2)
#define STM32F4_RCC_APB2ENR_IOPBEN		((uint32_t)1 << 3)
#define STM32F4_RCC_APB2ENR_IOPCEN		((uint32_t)1 << 4)
#define STM32F4_RCC_APB2ENR_IOPDEN		((uint32_t)1 << 5)

#define STM32F4_USART_SR_TXE				((uint32_t)1 << 7)
#define STM32F4_USART_SR_TC				((uint32_t)1 << 6)
#define STM32F4_USART_SR_RXNE				((uint32_t)1 << 5)

#define STM32F4_USART_CR1_UE				((uint32_t)1 << 13)
#define STM32F4_USART_CR1_M				((uint32_t)1 << 12)
#define STM32F4_USART_CR1_TXEIE			((uint32_t)1 << 7)
#define STM32F4_USART_CR1_TCIE			((uint32_t)1 << 6)
#define STM32F4_USART_CR1_RXNEIE			((uint32_t)1 << 5)
#define STM32F4_USART_CR1_TE				((uint32_t)1 << 3)
#define STM32F4_USART_CR1_RE				((uint32_t)1 << 2)

#define STM32F4_USART_CR2_CLKEN			((uint32_t)1 << 11)
#define STM32F4_USART_CR2_STOP_SFT		12
#define STM32F4_USART_CR2_STOP_MSK		((uint32_t)0x03 << \
											STM32_USART_CR2_STOP_SFT)

#define STM32F4_USART_CR3_CTSE			((uint32_t)1 << 9)
#define STM32F4_USART_CR3_RTSE			((uint32_t)1 << 8)
#define STM32F4_USART_CR3_HDSEL			((uint32_t)1 << 3)

#define AF_USART1 		((uint8_t)0x07)  /* USART1 Alternate Function mapping  */
#define AF_USART2 		((uint8_t)0x07)  /* USART1 Alternate Function mapping  */
#define AF_USART3 		((uint8_t)0x07)  /* USART1 Alternate Function mapping  */
#define AF_UART4 		((uint8_t)0x08)  /* USART1 Alternate Function mapping  */
#define AF_UART5 		((uint8_t)0x08)  /* USART1 Alternate Function mapping  */
#define AF_UART6 		((uint8_t)0x08)  /* USART1 Alternate Function mapping  */

typedef enum
{ 
  GPIO_Mode_IN   = 0x00, /*!< GPIO Input Mode */
  GPIO_Mode_OUT  = 0x01, /*!< GPIO Output Mode */
  GPIO_Mode_AF   = 0x02, /*!< GPIO Alternate function Mode */
  GPIO_Mode_AN   = 0x03  /*!< GPIO Analog Mode */
}GPIOMode_TypeDef;

typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;


typedef enum
{ 
  GPIO_Low_Speed     = 0x00, /*!< Low speed    */
  GPIO_Medium_Speed  = 0x01, /*!< Medium speed */
  GPIO_High_Speed    = 0x02, /*!< Fast speed   */
  GPIO_VeryHigh_Speed    = 0x03  /*!< High speed   */
}GPIOSpeed_TypeDef;

#define	 GPIO_PUPDR_NP				0
#define 	GPIO_PUPDR_PU				1
#define	GPIO_PUPDR_PD				2
#define	GPIO_PUPDR_RE				3
/** 
  * @brief  GPIO Configuration PullUp PullDown enumeration 
  */ 
typedef enum
{ 
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;
#define USART_NUM  2
static void (*stm32f4_usart_ontx[USART_NUM])(void *);
static void (*stm32f4_usart_onrx[USART_NUM])(void *, uint16_t data);
static void *stm32f4_usart_callback_param[USART_NUM];

static const uint8_t stm32f4_usart_irqn[USART_NUM] = 
{
#if USART_NUM >= 1
	USART1_IRQn, 
#endif
#if USART_NUM >= 2
	USART2_IRQn, 
#endif
#if USART_NUM >= 3
	USART3_IRQn, 
#endif
#if USART_NUM >= 4
	UART4_IRQn, 
#endif
#if USART_NUM >= 5
	UART5_IRQn
#endif
#if USART_NUM >= 6
	USART6_IRQn
#endif
#if USART_NUM >= 7
	UART7_IRQn, 
#endif
#if USART_NUM >= 8
	UART8_IRQn
#endif
};
static const USART_TypeDef *stm32f4_usarts[USART_NUM] = 
{
#if USART_NUM >= 1
	USART1, 
#endif
#if USART_NUM >= 2
	USART2, 
#endif
#if USART_NUM >= 3
	USART3, 
#endif
#if USART_NUM >= 4
	UART4, 
#endif
#if USART_NUM >= 5
	UART5
#endif
#if USART_NUM >= 6
	USART6
#endif
#if USART_NUM >= 7
	UART7
#endif
#if USART_NUM >= 8
	UART8
#endif
};

vsf_err_t stm32f4_usart_init(uint8_t index)
{
	uint8_t usart_idx = index & 0x0F;
	uint8_t remap_idx = (index >> 4) & 0x0F;
	
#if __VSF_DEBUG__
	if (usart_idx >= USART_NUM)
	{
		return VSFERR_NOT_SUPPORT;
	}
#endif
	switch (usart_idx)
	{
		case 0:
		{
			RCC->APB2ENR|=RCC_APB2ENR_USART1EN;
			switch (remap_idx)
			{
				case 0:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;

				   #ifdef USART00_RX_ENABLE	//PA_10
					GPIOA->AFR[1]&=~((uint32_t)(0x0f<<8));
					GPIOA->AFR[1]|=(uint32_t)(AF_USART1<<8);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<20);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<20);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<10);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<10);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<20);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<20);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<20);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<20);
				   #endif

				   #ifdef USART00_TX_ENABLE		//PA9
					GPIOA->AFR[1]&=~((uint32_t)(0x0f<<4));
					GPIOA->AFR[1]|=(uint32_t)(AF_USART1<<4);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<18);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<18);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<9);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<9);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<18);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<18);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<18);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<18);	   
				   #endif
				   
				   #ifdef USART00_CK_ENABLE		//PA8
				   	GPIOA->AFR[1]&=~((uint32_t)(0x0f));
					GPIOA->AFR[1]|=(uint32_t)(AF_USART1);
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<16);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<16);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<8);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<8);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<16);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<16);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<16);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<16);
				    #endif

				   #ifdef USART00_CTS_ENABLE//pA11
				    	GPIOA->AFR[1]&=~((uint32_t)(0x0f<<12));
					GPIOA->AFR[1]|=(uint32_t)(AF_USART1<<12);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<22);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<22);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<11);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<11);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<22);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<22);	
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<22);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<22);
					#endif
				   
					#ifdef USART00_RTS_ENABLE//PA12
				  GPIOA->AFR[1]&=~((uint32_t)(0x0f<<16));
					GPIOA->AFR[1]|=(uint32_t)(AF_USART1<<16);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<24);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<24);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<12);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<12);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<24);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<24);	
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<24);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<24);					
					#endif
				 
				   
					break;
				}
				case 1:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;

				   #ifdef USART00_RX_ENABLE	//PB_7
				    	GPIOB->AFR[0]&=~((uint32_t)(0x0f<<28));
					GPIOB->AFR[0]|=(uint32_t)(AF_USART1<<28);
					
					GPIOB->MODER&=~((uint32_t)GPIO_Mode_AN<<14);
					GPIOB->MODER|=((uint32_t)GPIO_Mode_AF<<14);

					GPIOB->OTYPER&=~((uint32_t)GPIO_OType_OD<<7);
					GPIOB->OTYPER|=((uint32_t)GPIO_OType_PP<<7);
					
					GPIOB-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<14);
					GPIOB-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<14);
					GPIOB->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<14);
					GPIOB->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<14);				  
					#endif

				   #ifdef USART00_TX_ENABLE		//PB6
				    	GPIOB->AFR[0]&=~((uint32_t)(0x0f<<24));
					GPIOB->AFR[0]|=(uint32_t)(AF_USART1<<24);
					
					GPIOB->MODER&=~((uint32_t)GPIO_Mode_AN<<12);
					GPIOB->MODER|=((uint32_t)GPIO_Mode_AF<<12);

					GPIOB->OTYPER&=~((uint32_t)GPIO_OType_OD<<6);
					GPIOB->OTYPER|=((uint32_t)GPIO_OType_PP<<6);
					
					GPIOB-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<12);
					GPIOB-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<12);
					GPIOB->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<12);
					GPIOB->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<12);	
				   #endif
					break;
				}
				default:
				{
					return VSFERR_NOT_SUPPORT;
				}
			}
			break;
		}
		case 1:
		{
			RCC->APB1ENR|=RCC_APB1ENR_USART2EN;
			switch (remap_idx)
			{
				case 0:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;

				   #ifdef USART01_RX_ENABLE	//PA3
					GPIOA->AFR[0]&=~((uint32_t)(0x0f<<12));
					GPIOA->AFR[0]|=(uint32_t)(AF_USART2<<12);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<6);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<6);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<3);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<3);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<6);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<6);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<6);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<6);	
				   #endif
				   #ifdef USART01_TX_ENABLE	//PA2
					GPIOA->AFR[0]&=~((uint32_t)(0x0f<<8));
					GPIOA->AFR[0]|=(uint32_t)(AF_USART2<<8);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<4);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<4);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<2);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<2);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<4);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<4);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<4);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<4);	
				   #endif				   

				   #ifdef USART01_RTS_ENABLE	//PA1
					GPIOA->AFR[0]&=~((uint32_t)(0x0f<<4));
					GPIOA->AFR[0]|=(uint32_t)(AF_USART2<<4);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<2);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<2);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<1);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<1);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<2);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<2);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<2);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<2);	
				   #endif	

				   #ifdef USART01_CTS_ENABLE	//PA0
					GPIOA->AFR[0]&=~((uint32_t)(0x0f));
					GPIOA->AFR[0]|=(uint32_t)(AF_USART2);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU);	
				   #endif	

				   #ifdef USART01_CK_ENABLE	//PA4
					GPIOA->AFR[0]&=~((uint32_t)(0x0f<<16));
					GPIOA->AFR[0]|=(uint32_t)(AF_USART2<<16);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<8);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<8);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<4);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<4);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<8);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<8);
					
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<8);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<8);	
				   #endif					   
				   	break;
				   	
				  }
				case 1:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;

					 #ifdef USART01_RX_ENABLE	//PD6
					GPIOD->AFR[0]&=~((uint32_t)(0x0f<<24));
					GPIOD->AFR[0]|=(uint32_t)(AF_USART2<<24);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<12);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<12);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<6);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<6);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<12);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<12);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<12);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<12);	
					 #endif

					 #ifdef USART01_TX_ENABLE	//PD5
					GPIOD->AFR[0]&=~((uint32_t)(0x0f<<20));
					GPIOD->AFR[0]|=(uint32_t)(AF_USART2<<20);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<10);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<10);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<5);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<5);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<10);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<10);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<10);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<10);	
					 #endif
					 break;

					 #ifdef USART01_CK_ENABLE	//PD7
					GPIOD->AFR[0]&=~((uint32_t)(0x0f<<28));
					GPIOD->AFR[0]|=(uint32_t)(AF_USART2<<28);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<14);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<14);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<7);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<7);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<14);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<14);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<14);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<14);
					 #endif
					 
					 #ifdef USART01_RTS_ENABLE	//PD4
					GPIOD->AFR[0]&=~((uint32_t)(0x0f<<16));
					GPIOD->AFR[0]|=(uint32_t)(AF_USART2<<16);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<8);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<8);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<4);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<4);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<8);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<8);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<8);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<8);
					 #endif		

					 #ifdef USART01_CTS_ENABLE	//PD3
					GPIOD->AFR[0]&=~((uint32_t)(0x0f<<12));
					GPIOD->AFR[0]|=(uint32_t)(AF_USART2<<12);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<6);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<6);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<3);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<3);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<6);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<6);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<6);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<6);
					 #endif				
					 break;
				}
				default :
				 return VSFERR_NOT_SUPPORT;
			}
			break;
		}
		case 2:
		{
			RCC->APB1ENR|=RCC_APB1ENR_USART3EN;
			switch (remap_idx)
			{
				case 0:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
			  	 #ifdef USART02_TX_ENABLE	//PB10
					GPIOB->AFR[0]&=~((uint32_t)(0x0f<<8));
					GPIOB->AFR[0]|=(uint32_t)(AF_USART3<<8);
					
					GPIOB->MODER&=~((uint32_t)GPIO_Mode_AN<<20);
					GPIOB->MODER|=((uint32_t)GPIO_Mode_AF<<20);

					GPIOB->OTYPER&=~((uint32_t)GPIO_OType_OD<<10);
					GPIOB->OTYPER|=((uint32_t)GPIO_OType_PP<<10);
					
					GPIOB-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<20);
					GPIOB-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<20);
					GPIOB->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<20);
					GPIOB->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<20);
				   #endif		

			  	 #ifdef USART02_RX_ENABLE	//PB11
					GPIOB->AFR[0]&=~((uint32_t)(0x0f<<12));
					GPIOB->AFR[0]|=(uint32_t)(AF_USART3<<12);
					
					GPIOB->MODER&=~((uint32_t)GPIO_Mode_AN<<22);
					GPIOB->MODER|=((uint32_t)GPIO_Mode_AF<<22);

					GPIOB->OTYPER&=~((uint32_t)GPIO_OType_OD<<11);
					GPIOB->OTYPER|=((uint32_t)GPIO_OType_PP<<11);
					
					GPIOB-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<22);
					GPIOB-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<22);
					GPIOB->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<22);
					GPIOB->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<22);
				   #endif		
				   
			  	 #ifdef USART02_CK_ENABLE	//PB12
					GPIOB->AFR[0]&=~((uint32_t)(0x0f<<16));
					GPIOB->AFR[0]|=(uint32_t)(AF_USART3<<16);
					
					GPIOB->MODER&=~((uint32_t)GPIO_Mode_AN<<24);
					GPIOB->MODER|=((uint32_t)GPIO_Mode_AF<<24);

					GPIOB->OTYPER&=~((uint32_t)GPIO_OType_OD<<12);
					GPIOB->OTYPER|=((uint32_t)GPIO_OType_PP<<12);
					
					GPIOB-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<24);
					GPIOB-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<24);
					GPIOB->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<24);
					GPIOB->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<24);
				   #endif
			  	 #ifdef USART02_CTS_ENABLE	//PB13
					GPIOB->AFR[0]&=~((uint32_t)(0x0f<<20));
					GPIOB->AFR[0]|=(uint32_t)(AF_USART3<<20);
					
					GPIOB->MODER&=~((uint32_t)GPIO_Mode_AN<<26);
					GPIOB->MODER|=((uint32_t)GPIO_Mode_AF<<26);

					GPIOB->OTYPER&=~((uint32_t)GPIO_OType_OD<<13);
					GPIOB->OTYPER|=((uint32_t)GPIO_OType_PP<<13);
					
					GPIOB-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<26);
					GPIOB-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<26);
					GPIOB->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<26);
					GPIOB->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<26);
				   #endif		
			  	 #ifdef USART02_RX_ENABLE	//PB14
					GPIOB->AFR[0]&=~((uint32_t)(0x0f<<24));
					GPIOB->AFR[0]|=(uint32_t)(AF_USART3<<24);
					
					GPIOB->MODER&=~((uint32_t)GPIO_Mode_AN<<28);
					GPIOB->MODER|=((uint32_t)GPIO_Mode_AF<<28);

					GPIOB->OTYPER&=~((uint32_t)GPIO_OType_OD<<14);
					GPIOB->OTYPER|=((uint32_t)GPIO_OType_PP<<14);
					
					GPIOB-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<28);
					GPIOB-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<28);
					GPIOB->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<28);
					GPIOB->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<28);
				   #endif					
				   break;
				}
				case 1:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
			  	 #ifdef USART02_TX_ENABLE	//PD8
					GPIOD->AFR[1]&=~((uint32_t)(0x0f));
					GPIOD->AFR[1]|=(uint32_t)(AF_USART3);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<16);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<16);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<8);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<8);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<16);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<16);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<16);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<16);
				   #endif		
			  	 #ifdef USART02_RX_ENABLE	//PD9
					GPIOD->AFR[1]&=~((uint32_t)(0x0f<<4));
					GPIOD->AFR[1]|=(uint32_t)(AF_USART3<<4);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<18);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<18);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<9);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<9);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<18);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<18);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<18);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<18);
				   #endif		
			  	 #ifdef USART02_CK_ENABLE	//PD10
					GPIOD->AFR[1]&=~((uint32_t)(0x0f<<8));
					GPIOD->AFR[1]|=(uint32_t)(AF_USART3<<8);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<20);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<20);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<10);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<10);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<20);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<20);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<20);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<20);
				   #endif	
			  	 #ifdef USART02_CTS_ENABLE	//PD11
					GPIOD->AFR[1]&=~((uint32_t)(0x0f<<12));
					GPIOD->AFR[1]|=(uint32_t)(AF_USART3<<12);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<22);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<22);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<11);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<11);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<22);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<22);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<22);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<22);
				   #endif	
			  	 #ifdef USART02_RTS_ENABLE	//PD12
					GPIOD->AFR[1]&=~((uint32_t)(0x0f<<16));
					GPIOD->AFR[1]|=(uint32_t)(AF_USART3<<16);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<24);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<24);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<12);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<12);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<24);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<24);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<24);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<24);
				   #endif	
					break;
				}
				case 2:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
		  	 	#ifdef USART02_TX_ENABLE	//PC10
					GPIOC->AFR[1]&=~((uint32_t)(0x0f<<8));
					GPIOC->AFR[1]|=(uint32_t)(AF_USART3<<8);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<20);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<20);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<10);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<10);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<20);
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<20);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<20);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<20);
				   #endif	

		  	 	#ifdef USART02_RX_ENABLE	//P11
					GPIOC->AFR[1]&=~((uint32_t)(0x0f<<12));
					GPIOC->AFR[1]|=(uint32_t)(AF_USART3<<12);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<22);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<22);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<11);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<11);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<22);
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<22);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<22);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<22);
				   #endif

		  	 	#ifdef USART02_CK_ENABLE	//P12
					GPIOC->AFR[1]&=~((uint32_t)(0x0f<<16));
					GPIOC->AFR[1]|=(uint32_t)(AF_USART3<<16);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<24);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<24);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<12);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<12);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<24);
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<24);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<24);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<24);
				   #endif
					break;
				}
				default :
					return VSFERR_NOT_SUPPORT;
			}
			break;
		}			
		case 3:
		{
			RCC->APB2ENR|=RCC_APB1ENR_UART4EN;
			switch (remap_idx)
			{
				case 0:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
			 	#ifdef USART03_TX_ENABLE	//PA0
					GPIOA->AFR[0]&=~((uint32_t)(0x0f));
					GPIOA->AFR[0]|=(uint32_t)(AF_UART4);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU);
				#endif	

				#ifdef USART03_RX_ENABLE	//PA1
					GPIOA->AFR[0]&=~((uint32_t)(0x0f)<<4);
					GPIOA->AFR[0]|=(uint32_t)(AF_UART4<<4);
					
					GPIOA->MODER&=~((uint32_t)GPIO_Mode_AN<<2);
					GPIOA->MODER|=((uint32_t)GPIO_Mode_AF<<2);

					GPIOA->OTYPER&=~((uint32_t)GPIO_OType_OD<<1);
					GPIOA->OTYPER|=((uint32_t)GPIO_OType_PP<<1);
					
					GPIOA-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<2);
					GPIOA-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<2);
					GPIOA->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<2);
					GPIOA->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<2);
				#endif	
					break;
				}
				case 1:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
					 #ifdef USART03_TX_ENABLE	//PC10
					GPIOC->AFR[1]&=~((uint32_t)(0x0f)<<8);
					GPIOC->AFR[1]|=(uint32_t)(AF_UART4<<8);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<20);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<20);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<10);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<10);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<20);
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<20);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<20);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<20);
				#endif
				
				#ifdef USART03_RX_ENABLE	//PC11
					GPIOC->AFR[1]&=~((uint32_t)(0x0f)<<12);
					GPIOC->AFR[1]|=(uint32_t)(AF_UART4<<12);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<22);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<22);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<11);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<11);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<22 );
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<22);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<22);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<22);
				#endif
					break;	
				}
				default :
						return VSFERR_NOT_SUPPORT;
			}
			break;
		}

		case 4:
		{
			RCC->APB1ENR|=RCC_APB1ENR_UART5EN;
			switch (remap_idx)
			{
				case 0:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
				#ifdef USART04_TX_ENABLE	//PC12
					GPIOC->AFR[1]&=~((uint32_t)(0x0f)<<16);
					GPIOC->AFR[1]|=(uint32_t)(AF_UART5<<16);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<24);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<24);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<12);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<12);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<24);
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<24);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<24);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<24);
				#endif
				#ifdef USART04_RX_ENABLE	//PD2
					GPIOD->AFR[0]&=~((uint32_t)(0x0f)<<8);
					GPIOD->AFR[0]|=(uint32_t)(AF_UART5<<8);
					
					GPIOD->MODER&=~((uint32_t)GPIO_Mode_AN<<4);
					GPIOD->MODER|=((uint32_t)GPIO_Mode_AF<<4);

					GPIOD->OTYPER&=~((uint32_t)GPIO_OType_OD<<2);
					GPIOD->OTYPER|=((uint32_t)GPIO_OType_PP<<2);
					
					GPIOD-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<4);
					GPIOD-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<4);
					GPIOD->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<4);
					GPIOD->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<4);
				#endif					
					break;
				}

				default :
					return VSFERR_NOT_SUPPORT;
			}
			break;			
		}
		case 5:
		{
			RCC->APB2ENR|=RCC_APB2ENR_USART6EN;
		
			switch (remap_idx)
			{
				case 0:
				{
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOGEN;
				#ifdef USART05_TX_ENABLE	//PC6
					GPIOC->AFR[0]&=~((uint32_t)(0x0f)<<24);
					GPIOC->AFR[0]|=(uint32_t)(AF_UART5<<24);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<12);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<12);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<6);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<6);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<12);
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<12);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<12);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<12);
				#endif	

				#ifdef USART05_RX_ENABLE	//PC7
					GPIOC->AFR[0]&=~((uint32_t)(0x0f)<<28);
					GPIOC->AFR[0]|=(uint32_t)(AF_UART5<<28);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<14);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<14);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<7);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<7);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<14);
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<14);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<14);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<14);
				#endif
				#ifdef USART05_CK_ENABLE	//PC8
					GPIOC->AFR[1]&=~((uint32_t)(0x0f));
					GPIOC->AFR[1]|=(uint32_t)(AF_UART5);
					
					GPIOC->MODER&=~((uint32_t)GPIO_Mode_AN<<16);
					GPIOC->MODER|=((uint32_t)GPIO_Mode_AF<<16);

					GPIOC->OTYPER&=~((uint32_t)GPIO_OType_OD<<8);
					GPIOC->OTYPER|=((uint32_t)GPIO_OType_PP<<8);
					
					GPIOC-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<16);
					GPIOC-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<16);
					GPIOC->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<16);
					GPIOC->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<16);
				#endif
				#ifdef USART05_RTS_ENABLE	//PG8
					GPIOG->AFR[1]&=~((uint32_t)(0x0f));
					GPIOG->AFR[1]|=(uint32_t)(AF_UART5);
					
					GPIOG->MODER&=~((uint32_t)GPIO_Mode_AN<<16);
					GPIOG->MODER|=((uint32_t)GPIO_Mode_AF<<16);

					GPIOG->OTYPER&=~((uint32_t)GPIO_OType_OD<<8);
					GPIOG->OTYPER|=((uint32_t)GPIO_OType_PP<<8);
					
					GPIOG-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<16);
					GPIOG-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<16);

					GPIOG->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<16);
					GPIOG->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<16);

				#endif
				#ifdef USART05_CTS_ENABLE	//PG13
				
					GPIOG->AFR[1]&=~((uint32_t)(0x0f)<<20);
					GPIOG->AFR[1]|=(uint32_t)(AF_UART5<<20);
					
					GPIOG->MODER&=~((uint32_t)GPIO_Mode_AN<<26);
					GPIOG->MODER|=((uint32_t)GPIO_Mode_AF<<26);

					GPIOG->OTYPER&=~((uint32_t)GPIO_OType_OD<<13);
					GPIOG->OTYPER|=((uint32_t)GPIO_OType_PP<<13);
					
					GPIOG-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<26);
					GPIOG-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<26);
					GPIOG->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<26);
					GPIOG->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<26);
				#endif				
					break;
				}
				case 1:
				{

					RCC->AHB1ENR|=RCC_AHB1ENR_GPIOGEN;
				#ifdef USART05_TX_ENABLE	//PG14
					GPIOG->AFR[1]&=~((uint32_t)(0x0f)<<24);
					GPIOG->AFR[1]|=(uint32_t)(AF_UART5<<24);
					
					GPIOG->MODER&=~((uint32_t)GPIO_Mode_AN<<28);
					GPIOG->MODER|=((uint32_t)GPIO_Mode_AF<<28);

					GPIOG->OTYPER&=~((uint32_t)GPIO_OType_OD<<14);
					GPIOG->OTYPER|=((uint32_t)GPIO_OType_PP<<14);
					
					GPIOG-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<28);
					GPIOG-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<28);
					GPIOG->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<28);
					GPIOG->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<28);
				#endif
				
				#ifdef USART05_RX_ENABLE	//PG9
					GPIOG->AFR[1]&=~((uint32_t)(0x0f)<<4);
					GPIOG->AFR[1]|=(uint32_t)(AF_UART5<<4);
					
					GPIOG->MODER&=~((uint32_t)GPIO_Mode_AN<<18);
					GPIOG->MODER|=((uint32_t)GPIO_Mode_AF<<18);

					GPIOG->OTYPER&=~((uint32_t)GPIO_OType_OD<<9);
					GPIOG->OTYPER|=((uint32_t)GPIO_OType_PP<<9);
					
					GPIOG-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<18);
					GPIOG-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<18);
					GPIOG->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<18);
					GPIOG->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<18);
				#endif
				#ifdef USART05_CK_ENABLE	//PG7
					GPIOG->AFR[0]&=~((uint32_t)(0x0f)<<28);
					GPIOG->AFR[0]|=(uint32_t)(AF_UART5<<28);
					
					GPIOG->MODER&=~((uint32_t)GPIO_Mode_AN<<14);
					GPIOG->MODER|=((uint32_t)GPIO_Mode_AF<<14);

					GPIOG->OTYPER&=~((uint32_t)GPIO_OType_OD<<7);
					GPIOG->OTYPER|=((uint32_t)GPIO_OType_PP<<7);
					
					GPIOG-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<14);
					GPIOG-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<14);
					GPIOG->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<14);
					GPIOG->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<14);
				#endif
				#ifdef USART05_RTS_ENABLE	//PG12
					GPIOG->AFR[1]&=~((uint32_t)(0x0f)<<16);
					GPIOG->AFR[1]|=(uint32_t)(AF_UART5<<16);
					
					GPIOG->MODER&=~((uint32_t)GPIO_Mode_AN<<24);
					GPIOG->MODER|=((uint32_t)GPIO_Mode_AF<<24);

					GPIOG->OTYPER&=~((uint32_t)GPIO_OType_OD<<12);
					GPIOG->OTYPER|=((uint32_t)GPIO_OType_PP<<12);
					
					GPIOG-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<24);
					GPIOG-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<24);
					GPIOG->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<24);
					GPIOG->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<24);
				#endif
				#ifdef USART05_RTS_ENABLE	//PG15
					GPIOG->AFR[1]&=~((uint32_t)(0x0f)<<28);
					GPIOG->AFR[1]|=(uint32_t)(AF_UART5<<28);
					
					GPIOG->MODER&=~((uint32_t)GPIO_Mode_AN<<30);
					GPIOG->MODER|=((uint32_t)GPIO_Mode_AF<<30);

					GPIOG->OTYPER&=~((uint32_t)GPIO_OType_OD<<15);
					GPIOG->OTYPER|=((uint32_t)GPIO_OType_PP<<15);
					
					GPIOG-> OSPEEDR&= ~((uint32_t)GPIO_VeryHigh_Speed<<30);
					GPIOG-> OSPEEDR|= ((uint32_t)GPIO_High_Speed<<30);
					GPIOG->PUPDR&=~((uint32_t)GPIO_PUPDR_RE<<30);
					GPIOG->PUPDR|=((uint32_t)GPIO_PUPDR_PU<<30);
				#endif
					break;
				}
				default :
					return VSFERR_NOT_SUPPORT;
			}
			break;
		}
		default :
			return VSFERR_NOT_SUPPORT;
	
	}
	
}
#define USART00_ENABLE 	1
#define	USART00_TX_ENABLE	1
#define USART00_RX_ENABLE	1

vsf_err_t stm32f4_usart_config(uint8_t index, uint32_t baudrate, uint32_t mode)
{
	USART_TypeDef *usart;
	uint8_t usart_idx = index & 0x0F;
	uint32_t cr1 = 0, cr2 = 0, cr3 = 0;
	struct stm32f4_info_t *info;
	uint32_t module_hz;
	uint32_t mantissa, fraction, divider;
	
#if __VSF_DEBUG__
	if (usart_idx >= USART_NUM)
	{
		return VSFERR_NOT_SUPPORT;
	}
#endif
	usart = (USART_TypeDef *)stm32f4_usarts[usart_idx];
	
//	if (stm32_interface_get_info(&info))
//	{
//		return VSFERR_FAIL;
//	}
	
	cr1 = usart->CR1 & (STM32F4_USART_CR1_TCIE | STM32F4_USART_CR1_RXNEIE);
	switch (usart_idx)
	{
	#if USART00_ENABLE || USART10_ENABLE
	case 0:
	
		module_hz = 84*1000*1000;//info->apb2_periph_freq_hz;
		#if (USART00_ENABLE && USART00_TX_ENABLE) || \
			(USART10_ENABLE && USART10_TX_ENABLE)
		cr1 |= STM32F4_USART_CR1_TE;
		#endif
		#if (USART00_ENABLE && USART00_RX_ENABLE) || \
			(USART10_ENABLE && USART10_RX_ENABLE)
		cr1 |= STM32F4_USART_CR1_RE;
		#endif
		#if (USART00_ENABLE && USART00_CK_ENABLE) || \
			(USART10_ENABLE && USART10_CK_ENABLE)
		cr2 |= STM32_USART_CR2_CLKEN;
		#endif
		#if (USART00_ENABLE && USART00_CTS_ENABLE) || \
			(USART10_ENABLE && USART10_CTS_ENABLE)
		cr3 |= STM32_USART_CR3_CTSE;
		#endif
		#if (USART00_ENABLE && USART00_RTS_ENABLE) || \
			(USART10_ENABLE && USART10_RTS_ENABLE)
		cr3 |= STM32_USART_CR3_RTSE;
		#endif
		break;
	#endif
	#if USART01_ENABLE || USART11_ENABLE
	case 1:
	#endif
	#if USART11_ENABLE||USART01_ENABLE||USART02_ENABLE || USART12_ENABLE || USART32_ENABLE
	case 2:
		module_hz = info->apb1_freq_hz;
		#if  	(USART01_ENABLE && USART01_TX_ENABLE) || \
			(USART11_ENABLE && USART11_TX_ENABLE) ||\
			(USART02_ENABLE && USART02_TX_ENABLE) || \
			(USART12_ENABLE && USART12_TX_ENABLE) || \
			(USART32_ENABLE && USART32_TX_ENABLE)
		cr1 |= STM32_USART_CR1_TE;
		#endif
		#if 	(USART01_ENABLE && USART01_RX_ENABLE) || \
			(USART11_ENABLE && USART11_RX_ENABLE) ||\
			(USART02_ENABLE && USART02_RX_ENABLE) || \
			(USART12_ENABLE && USART12_RX_ENABLE) || \
			(USART32_ENABLE && USART32_RX_ENABLE)
		cr1 |= STM32_USART_CR1_RE;
		#endif
		#if 	(USART01_ENABLE && USART01_CTS_ENABLE) || \
			(USART11_ENABLE && USART11_CTS_ENABLE) ||\
			(USART02_ENABLE && USART02_CK_ENABLE)   || \
			(USART12_ENABLE && USART12_CK_ENABLE)   || \
			(USART32_ENABLE && USART32_CK_ENABLE)
		cr2 |= STM32_USART_CR2_CLKEN;
		#endif
		#if 	(USART01_ENABLE && USART01_CTS_ENABLE) || \
			(USART11_ENABLE && USART11_CTS_ENABLE) ||\
			(USART02_ENABLE && USART02_CTS_ENABLE) || \
			(USART12_ENABLE && USART12_CTS_ENABLE) || \
			(USART32_ENABLE && USART32_CTS_ENABLE)
		cr3 |= STM32_USART_CR3_CTSE;
		#endif
		#if 	 (USART01_ENABLE && USART01_RTS_ENABLE) || \
			(USART11_ENABLE && USART11_RTS_ENABLE) ||\
			(USART02_ENABLE && USART02_RTS_ENABLE) || \
			(USART12_ENABLE && USART12_RTS_ENABLE) || \
			(USART32_ENABLE && USART32_RTS_ENABLE)
		cr3 |= STM32_USART_CR3_RTSE;
		#endif
		break;
	#endif
	#if USART03_ENABLE
	case 3:
	#endif
	#if USART04_ENABLE||USART03_ENABLE
	case 4:
		module_hz = info->apb1_freq_hz;
		#if 	(USART03_ENABLE && USART03_TX_ENABLE)||\
			(USART13_ENABLE && USART13_TX_ENABLE)||\
			(USART04_ENABLE && USART04_TX_ENABLE)||\
			(USART14_ENABLE && USART14_TX_ENABLE)
		cr1 |= STM32_USART_CR1_TE;
		#endif
		#if 	(USART03_ENABLE && USART03_RX_ENABLE)||\
			(USART13_ENABLE && USART13_RX_ENABLE)||\
			(USART04_ENABLE && USART04_RX_ENABLE)||\
			(USART14_ENABLE && USART14_RX_ENABLE)
		cr1 |= STM32_USART_CR1_RE;
		#endif
		break;
	#endif
	#if USART05_ENABLE
		case 5:
		module_hz = info->apb2_freq_hz;
		#if (USART05_ENABLE && USART05_TX_ENABLE) || \
			(USART15_ENABLE && USART15_TX_ENABLE)
		cr1 |= STM32_USART_CR1_TE;
		#endif
		#if (USART05_ENABLE && USART05_RX_ENABLE) || \
			(USART15_ENABLE && USART15_RX_ENABLE)
		cr1 |= STM32_USART_CR1_RE;
		#endif
		#if (USART05_ENABLE && USART05_CK_ENABLE) || \
			(USART15_ENABLE && USART15_CK_ENABLE)
		cr2 |= STM32_USART_CR2_CLKEN;
		#endif
		#if (USART05_ENABLE && USART05_CTS_ENABLE) || \
			(USART15_ENABLE && USART15_CTS_ENABLE)
		cr3 |= STM32_USART_CR3_CTSE;
		#endif
		#if (USART05_ENABLE && USART05_RTS_ENABLE) || \
			(USART15_ENABLE && USART15_RTS_ENABLE)
		cr3 |= STM32_USART_CR3_RTSE;
		#endif
		break;
	#endif
	#if USART06_ENABLE
		case 6:		
	#endif
	#if USART07_ENABLE||USART06_ENABLE
		case 7:	
		module_hz = info->apb1_freq_hz;
		#if 	(USART06_ENABLE && USART06_TX_ENABLE)||\
			(USART16_ENABLE && USART16_TX_ENABLE)||\
			(USART07_ENABLE && USART07_TX_ENABLE)||\
			(USART17_ENABLE && USART17_TX_ENABLE)
		cr1 |= STM32_USART_CR1_TE;
		#endif
		#if 	(USART06_ENABLE && USART06_RX_ENABLE)||\
			(USART16_ENABLE && USART16_RX_ENABLE)||\
			(USART07_ENABLE && USART07_RX_ENABLE)||\
			(USART17_ENABLE && USART17_RX_ENABLE)
		cr1 |= STM32_USART_CR1_RE;
		#endif
		break;
	#endif
	default:
		return VSFERR_NOT_SUPPORT;
	}
	

	cr1 |= ((uint32_t)mode << 9) & 0x0600;
	cr1|=0x0000400c;
	cr2 |= ((uint32_t)mode << 7) & 0x3600;
	
	// baudrate
	divider = module_hz / baudrate;
	mantissa = divider / 16;
	fraction = divider - mantissa * 16;
	usart->BRR = (mantissa << 4) | fraction;
	usart->SR=0x00000000;
	usart->CR1 = cr1;
	usart->CR2 = cr2;
	usart->CR3 = cr3;
	usart->CR1 |= STM32F4_USART_CR1_UE;
	return VSFERR_NONE;	
}


vsf_err_t stm32f4_usart_fini(uint8_t index)
{
	USART_TypeDef *usart;
	uint8_t usart_idx = index & 0x0F;
	uint8_t remap_idx = (index >> 4) & 0x0F;
	
#if __VSF_DEBUG__
	if (usart_idx >= USART_NUM)
	{
		return VSFERR_NOT_SUPPORT;
	}
#endif
	usart = (USART_TypeDef *)stm32f4_usarts[usart_idx];
	
	usart->CR1 = 0;
	switch(usart_idx)
	{
	#if USART00_ENABLE || USART10_ENABLE	
		case 0:
		RCC->APB2ENR&=(uint32_t)(~RCC_APB2ENR_USART1EN);
		switch(remap_idx)
		{
			case 0:
			break;
			case 1:
			break;
		}
		break;
	#endif	
		case 1:
		break;
		case 2:
		break;
		case 3:
		break;
		case 4:
		break;
		case 5:
		break;
		case 6:
		break;
		case 7:
		break;
	}
}


vsf_err_t stm32f4_usart_config_callback(uint8_t index, uint32_t int_priority,
				void *p, void (*ontx)(void *), void (*onrx)(void *, uint16_t))
{
	USART_TypeDef *usart;
	uint8_t usart_idx = index & 0x0F;
	uint32_t cr1 = 0;
	uint8_t irqn;
	
#if __VSF_DEBUG__
	if (usart_idx >= USART_NUM)
	{
		return VSFERR_NOT_SUPPORT;
	}
#endif
	usart = (USART_TypeDef *)stm32f4_usarts[usart_idx];
	irqn = stm32f4_usart_irqn[index];
	
	stm32f4_usart_ontx[usart_idx] = ontx;
	stm32f4_usart_onrx[usart_idx] = onrx;
	stm32f4_usart_callback_param[usart_idx] = p;
	if (ontx != NULL)
	{
		cr1 |= STM32F4_USART_CR1_TCIE;
	}
	if (onrx != NULL)
	{
		cr1 |= STM32F4_USART_CR1_RXNEIE;
	}
	usart->CR1 &= ~(STM32F4_USART_CR1_TCIE | STM32F4_USART_CR1_RXNEIE);
	usart->CR1 |= cr1;
	
	if ((ontx != NULL) || (onrx != NULL))
	{
		NVIC->IP[irqn] = int_priority;
		NVIC->ISER[irqn >> 0x05] = 1UL << (irqn & 0x1F);
	}
	else
	{
		NVIC->ICER[irqn >> 0x05] = 1UL << (irqn & 0x1F);
	}
	return VSFERR_NONE;
}

vsf_err_t stm32f4_usart_tx(uint8_t index, uint16_t data)
{
	USART_TypeDef *usart;
	uint8_t usart_idx = index & 0x0F;
	
#if __VSF_DEBUG__
	if (usart_idx >= USART_NUM)
	{
		return VSFERR_NOT_SUPPORT;
	}
#endif
	usart = (USART_TypeDef *)stm32f4_usarts[usart_idx];
	
	usart->DR = data;
	return VSFERR_NONE;
}

uint16_t stm32f4_usart_rx(uint8_t index)
{
	USART_TypeDef *usart;
	uint8_t usart_idx = index & 0x0F;
	
#if __VSF_DEBUG__
	if (usart_idx >= USART_NUM)
	{
		return 0;
	}
#endif
	usart = (USART_TypeDef *)stm32f4_usarts[usart_idx];
	
	return usart->DR;
}

//#if USART0_INT_EN && (USART00_ENABLE || USART10_ENABLE)
#if 1
ROOTFUNC void USART1_IRQHandler(void)
{
	if ((stm32f4_usart_onrx[0] != NULL) && (USART1->SR & STM32F4_USART_SR_RXNE))
	{
		stm32f4_usart_onrx[0](stm32f4_usart_callback_param[0], USART1->DR);
	}
	if ((stm32f4_usart_ontx[0] != NULL) && (USART1->SR & STM32F4_USART_SR_TC))
	{
		stm32f4_usart_ontx[0](stm32f4_usart_callback_param[0]);
		USART1->SR &= ~STM32F4_USART_SR_TC;
	}
}
#endif

#if USART1_INT_EN && (USART01_ENABLE || USART11_ENABLE)
ROOTFUNC void USART2_IRQHandler(void)
{
	if ((stm32f4_usart_onrx[1] != NULL) && (USART2->SR & STM32_USART_SR_RXNE))
	{
		stm32f4_usart_onrx[1](stm32f4_usart_callback_param[1], USART2->DR);
	}
	if ((stm32f4_usart_ontx[1] != NULL) && (USART2->SR & STM32_USART_SR_TC))
	{
		stm32f4_usart_ontx[1](stm32f4_usart_callback_param[1]);
		USART2->SR &= ~STM32_USART_SR_TC;
	}
}
#endif

#if USART2_INT_EN && (USART02_ENABLE || USART12_ENABLE || USART32_ENABLE)
ROOTFUNC void USART3_IRQHandler(void)
{
	if ((stm32f4_usart_onrx[2] != NULL) && (USART3->SR & STM32_USART_SR_RXNE))
	{
		stm32f4_usart_onrx[2](stm32f4_usart_callback_param[2], USART3->DR);
	}
	if ((stm32f4_usart_ontx[2] != NULL) && (USART3->SR & STM32_USART_SR_TC))
	{
		stm32f4_usart_ontx[2](stm32f4_usart_callback_param[2]);
		USART3->SR &= ~STM32_USART_SR_TC;
	}
}
#endif

#if USART3_INT_EN && (USART03_ENABLE || USART13_ENABLE)
ROOTFUNC void USART4_IRQHandler(void)
{
	if ((stm32f4_usart_onrx[3] != NULL) && (UART4->SR & STM32_USART_SR_RXNE))
	{
		stm32f4_usart_onrx[3](stm32f4_usart_callback_param[3], UART4->DR);
	}
	if ((stm32f4_usart_ontx[3] != NULL) && (UART4->SR & STM32_USART_SR_TC))
	{
		stm32f4_usart_ontx[3](stm32f4_usart_callback_param[3]);
		UART4->SR &= ~STM32_USART_SR_TC;
	}
}
#endif

#if USART4_INT_EN && (USART04_ENABLE || USART14_ENABLE)
ROOTFUNC void USART5_IRQHandler(void)
{
	if ((stm32f4_usart_onrx[4] != NULL) && (UART5->SR & STM32_USART_SR_RXNE))
	{
		stm32f4_usart_onrx[4](stm32f4_usart_callback_param[4], UART5->DR);
	}
	if ((stm32f4_usart_ontx[4] != NULL) && (UART5->SR & STM32_USART_SR_TC))
	{
		stm32f4_usart_ontx[4](stm32f4_usart_callback_param[4]);
		UART5->SR &= ~STM32_USART_SR_TC;
	}
}
#endif

#if USART5_INT_EN && (USART05_ENABLE || USART15_ENABLE)
ROOTFUNC void USART6_IRQHandler(void)
{
	if ((stm32f4_usart_onrx[5] != NULL) && (USART6->SR & STM32_USART_SR_RXNE))
	{
		stm32f4_usart_onrx[5](stm32f4_usart_callback_param[5], UART6->DR);
	}
	if ((stm32f4_usart_ontx[5] != NULL) && (USART6->SR & STM32_USART_SR_TC))
	{
		stm32f4_usart_ontx[5](stm32f4_usart_callback_param[5]);
		USART6->SR &= ~STM32_USART_SR_TC;
	}
}
#endif

#if USART6_INT_EN && (USART06_ENABLE || USART16_ENABLE)
ROOTFUNC void USART7_IRQHandler(void)
{
	if ((stm32f4_usart_onrx[6] != NULL) && (UART7->SR & STM32_USART_SR_RXNE))
	{
		stm32f4_usart_onrx[6](stm32f4_usart_callback_param[6], UART7->DR);
	}
	if ((stm32f4_usart_ontx[6] != NULL) && (UART7->SR & STM32_USART_SR_TC))
	{
		stm32f4_usart_ontx[6](stm32f4_usart_callback_param[6]);
		UART7->SR &= ~STM32_USART_SR_TC;
	}
}
#endif

#if USART7_INT_EN && (USART07_ENABLE || USART17_ENABLE)
ROOTFUNC void USART8_IRQHandler(void)
{
	if ((stm32f4_usart_onrx[7] != NULL) && (UART8->SR & STM32_USART_SR_RXNE))
	{
		stm32f4_usart_onrx[7](stm32f4_usart_callback_param[7], UART8->DR);
	}
	if ((stm32f4_usart_ontx[7] != NULL) && (UART8->SR & STM32_USART_SR_TC))
	{
		stm32f4_usart_ontx[7](stm32f4_usart_callback_param[7]);
		UART8->SR &= ~STM32_USART_SR_TC;
	}
}
#endif

