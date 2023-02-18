
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPDIFRX_H
#define __SPDIFRX_H

#include "stm32f7xx_hal.h"

/*
 * define the S/PDIF Input on STM32F7 Discovery board
 */
#define DISCOVERY_SPDIFRX_CLK_ENABLE()           __HAL_RCC_SPDIFRX_CLK_ENABLE()

/* Definition for SPDIF IN0 pin */
#define DISCOVERY_SPDIFRX	                     SPDIFRX
#define DISCOVERY_SPDIFRX_GPIO_CLK_ENABLE()   	 __HAL_RCC_GPIOD_CLK_ENABLE()
#define DISCOVERY_SPDIFRX_PIN		             GPIO_PIN_7
#define DISCOVERY_SPDIFRX_PORT				     GPIOD
#define DISCOVERY_SPDIFRX_AF			         GPIO_AF8_SPDIFRX

#define DISCOVERY_SPDIFRX_DMAx_CLK_ENABLE()      __HAL_RCC_DMA2_CLK_ENABLE()
#define DISCOVERY_SPDIFRX_DMAx_STREAM            DMA2_Stream7
#define DISCOVERY_SPDIFRX_DMAx_CHANNEL           DMA_CHANNEL_0
#define DISCOVERY_SPDIFRX_DMAx_IRQ               DMA2_Stream7_IRQn
#define DISCOVERY_SPDIFRX_DMAx_PERIPH_DATA_SIZE  DMA_PDATAALIGN_HALFWORD
#define DISCOVERY_SPDIFRX_DMAx_MEM_DATA_SIZE     DMA_MDATAALIGN_HALFWORD

#define DISCOVERY_SPDIFRX_DMAx_IRQ				 DMA2_Stream7_IRQn
#define DISCOVERY_SPDIFRX_DMAx_IRQHandler        DMA2_Stream7_IRQHandler
#define DISCOVERY_SPDIFRX_IRQ					 SPDIF_RX_IRQn
#define DISCOVERY_SPDIFRX_IRQHandler			 SPDIF_RX_IRQHandler
#define DISCOVERY_SPDIFRX_IRQ_PREPRIO	         ((uint32_t)5)   /* Select the preemption priority level(0 is the highest) */
/*
 *
 */
void HAL_SPDIFRX_MspInit(SPDIFRX_HandleTypeDef *hspdif);

/*
 * Exported functions
 */

HAL_StatusTypeDef BSP_SPDIFRX_Init(void);

void SPDIFRX_Start_IT(void);
int  SPDIFRX_Loop(void);

#endif
