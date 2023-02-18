
#include "stm32746g_discovery.h"
#include "spdifrx.h"

#include "usbd_audio_if.h"
#include "stm32746g_discovery_audio.h"
#include "usbd_audio.h"

SPDIFRX_HandleTypeDef hSPDIFRX_IN0;

#define SPDIF_SAMPLE_NUM	(192*2 * 40)		//check linker script for enough malloc space, here as 1-byte words!
//but above 40 it does not work anymore !!!

static uint32_t *SPDIFIN0_RxBuf = NULL;			//use malloc, pls. see _Min_Heap_Size  = 0x4000; in LinkerScript.ld
//static uint32_t SPDIFIN0_CsBuf[192];

/* global */ int gUSBInit_Done = 0;

/**
  * @brief SPDIFRX MSP Init
  * @param hspdif: SPDIFRX handle
  * @retval None
  */
void HAL_SPDIFRX_MspInit(SPDIFRX_HandleTypeDef *hspdif)
{
	GPIO_InitTypeDef  gpio_init_structure;

	if(hspdif->Instance == DISCOVERY_SPDIFRX)
	{
		HAL_SPDIFRX_MspDeInit(hspdif);

		DISCOVERY_SPDIFRX_CLK_ENABLE();

		/*** Configure the GPIOs ***/
		/* Enable GPIO clock */
		DISCOVERY_SPDIFRX_GPIO_CLK_ENABLE();

		/* Configure SPDIFRX_IN0 as alternate function */
		gpio_init_structure.Pin = DISCOVERY_SPDIFRX_PIN;
		gpio_init_structure.Mode = GPIO_MODE_AF_PP;
		gpio_init_structure.Pull = GPIO_NOPULL;
		gpio_init_structure.Speed = GPIO_SPEED_FAST;
		gpio_init_structure.Alternate = DISCOVERY_SPDIFRX_AF;
		HAL_GPIO_Init(DISCOVERY_SPDIFRX_PORT, &gpio_init_structure);

		//DISCOVERY_SPDIFRX_DMAx_CLK_ENABLE();

		/*
		 * Configure DMA
		 */

		/* Enable and set SPDIF DMA Interrupt to a lower priority */
		//HAL_NVIC_SetPriority(DISCOVERY_SPDIFRX_DMAx_IRQ, DISCOVERY_SPDIFRX_IRQ_PREPRIO, 0);
		//HAL_NVIC_EnableIRQ(DISCOVERY_SPDIFRX_DMAx_IRQ);

		HAL_NVIC_SetPriority(DISCOVERY_SPDIFRX_IRQ, DISCOVERY_SPDIFRX_IRQ_PREPRIO, 0);
		HAL_NVIC_EnableIRQ(DISCOVERY_SPDIFRX_IRQ);
	}
}

void HAL_SPDIFRX_MspDeInit(SPDIFRX_HandleTypeDef* hspdifrx)
{

  if(hspdifrx->Instance == DISCOVERY_SPDIFRX)
  {
  /* USER CODE BEGIN SPDIFRX_MspDeInit 0 */

  /* USER CODE END SPDIFRX_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPDIFRX_CLK_DISABLE();

    /**SPDIFRX GPIO Configuration
    PD7     ------> SPDIFRX_IN0
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_7);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hspdifrx->hdmaDrRx);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(SPDIF_RX_IRQn);

  }
  /* USER CODE BEGIN SPDIFRX_MspDeInit 1 */

  /* USER CODE END SPDIFRX_MspDeInit 1 */
}

HAL_StatusTypeDef BSP_SPDIFRX_Init(void)
{
	HAL_StatusTypeDef errCode;

	RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;

	HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);

	/* configure SPDIFRX_CLK, PLLI2S, using PLLI2SP */
	/* PLLI2S_VCO: VCO_429M
	   I2S_CLK(first level) = PLLI2S_VCO/PLLI2SQ = 429/2 = 214.5 Mhz
	   I2S_CLK_x = I2S_CLK(first level)/PLLI2SDIVQ = 214.5/19 = 11.289 Mhz */
	rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SPDIFRX;
	rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 192;
	rcc_ex_clk_init_struct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
	rcc_ex_clk_init_struct.PLLI2S.PLLI2SR = 2;
	rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 2;
	rcc_ex_clk_init_struct.PLLI2SDivQ = 1;
	HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);

	//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	//HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/*
	 * Initialize the SPDIFRX_Handle
	 */

	hSPDIFRX_IN0.Instance 				= DISCOVERY_SPDIFRX;    	 /* SPDIFRX registers base address */

	hSPDIFRX_IN0.Init.ChannelSelection 	= SPDIFRX_CHANNEL_A;         /* SPDIFRX communication parameters */
	hSPDIFRX_IN0.Init.StereoMode 		= SPDIFRX_STEREOMODE_ENABLE;
	hSPDIFRX_IN0.Init.InputSelection 	= SPDIFRX_INPUT_IN0;
	hSPDIFRX_IN0.Init.Retries 			= SPDIFRX_MAXRETRIES_NONE;
	hSPDIFRX_IN0.Init.WaitForActivity 	= SPDIFRX_WAITFORACTIVITY_ON;
	hSPDIFRX_IN0.Init.DataFormat 		= SPDIFRX_DATAFORMAT_32BITS;
	hSPDIFRX_IN0.Init.PreambleTypeMask 	= SPDIFRX_PREAMBLETYPEMASK_OFF;
	hSPDIFRX_IN0.Init.ChannelStatusMask = SPDIFRX_CHANNELSTATUS_OFF;
	hSPDIFRX_IN0.Init.ValidityBitMask 	= SPDIFRX_VALIDITYMASK_OFF;
	hSPDIFRX_IN0.Init.ParityErrorMask 	= SPDIFRX_PARITYERRORMASK_OFF;

	if ( ! SPDIFIN0_RxBuf)
		SPDIFIN0_RxBuf = (uint32_t *)malloc(SPDIF_SAMPLE_NUM);

	hSPDIFRX_IN0.pRxBuffPtr 			= SPDIFIN0_RxBuf;	/* Pointer to SPDIFRX Rx transfer buffer */

	//hSPDIFRX_IN0.pCsBuffPtr 			= SPDIFIN0_CsBuf;	/* Pointer to SPDIFRX Cs transfer buffer */

	hSPDIFRX_IN0.RxXferSize 			= 192;				/* SPDIFRX Rx transfer size */

	hSPDIFRX_IN0.RxXferCount			= 192;  			/* SPDIFRX Rx transfer counter
	                                              	  	  	   (This field is initialized at the
	                                               	   	   	   same value as transfer size at the
	                                               	   	   	   beginning of the transfer and
	                                               	   	   	   decremented when a sample is received.
	                                               	   	   	   NbSamplesReceived = RxBufferSize-RxBufferCount) */

	hSPDIFRX_IN0.CsXferSize  = 192; 						/* SPDIFRX Cs transfer size */

	hSPDIFRX_IN0.CsXferCount = 192;		  					/* SPDIFRX Cs transfer counter
	                                              	  	  	   (This field is initialized at the
	                                               	   	   	   same value as transfer size at the
	                                               	   	   	   beginning of the transfer and
	                                               	   	   	   decremented when a sample is received.
	                                               	   	   	   NbSamplesReceived = RxBufferSize-RxBufferCount) */

	hSPDIFRX_IN0.hdmaCsRx = NULL; 							/* SPDIFRX EC60958_channel_status and user_information DMA handle parameters */

	hSPDIFRX_IN0.hdmaDrRx = NULL;							/* SPDIFRX Rx DMA handle parameters */

	hSPDIFRX_IN0.Lock = 0;         							/* SPDIFRX locking object */

	hSPDIFRX_IN0.State = HAL_SPDIFRX_STATE_RESET;    		/* SPDIFRX communication state */

	hSPDIFRX_IN0.ErrorCode = HAL_SPDIFRX_ERROR_NONE;

	/*
	 * Call the HAL init function
	 */
	errCode = HAL_SPDIFRX_Init(&hSPDIFRX_IN0);

	return errCode;
}


void SPDIF_RX_IRQHandler(void)
{
	/* this has a side effect: if USB audio - this corrupts the sound ! */
    HAL_SPDIFRX_IRQHandler(&hSPDIFRX_IN0);
}


void SPDIFRX_Start_IT(void)
{
	//HAL_SPDIFRX_ReceiveDataFlow_IT(&hSPDIFRX_IN0, SPDIFIN0_RxBuf, AUDIO_OUT_PACKET/4);
}

int SPDIFRX_Loop(void)
{
	static int playStarted = 0;

	if ( ! playStarted)
	{
		if (HAL_SPDIFRX_ReceiveDataFlow_IT(&hSPDIFRX_IN0, SPDIFIN0_RxBuf, SPDIF_SAMPLE_NUM) == HAL_OK)
		{
			/* assuming 32bit, with2x16bit for Channel A, B in sample word */
			if ( ! playStarted)
			{
				while (hSPDIFRX_IN0.RxXferCount > SPDIF_SAMPLE_NUM / 2) ;

				BSP_AUDIO_OUT_Play((uint16_t *)SPDIFIN0_RxBuf, (SPDIF_SAMPLE_NUM - 192) * 4);
				playStarted = 1;
			}
		}
	}

	while (hSPDIFRX_IN0.RxXferCount > 192) ;

	__disable_irq();

	hSPDIFRX_IN0.RxXferCount = SPDIF_SAMPLE_NUM;
	hSPDIFRX_IN0.pRxBuffPtr  = SPDIFIN0_RxBuf;

	__enable_irq();

	return 1;
}
