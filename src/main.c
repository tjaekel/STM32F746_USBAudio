/**
  ******************************************************************************
  * @file    USB_Device/AUDIO_Standalone/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   USB device AUDIO demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "main.h"
#include "WM.h"

#if 1
#include "logo.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;

uint8_t GUI_Initialized = 0;
TIM_HandleTypeDef TimHandle;
uint32_t uwPrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
static void Toggle_Leds(void);

static void SPDIF_Interface(int mode);
#ifdef USB_AUDIO
static void USB_Interface(int mode);
#if 0
extern void USBD_AUDIO_Set(int mode);
#endif
#endif

#if 1
static void MIC_Interface(int mode);
#endif

void BSP_Background(void);
//extern void MainTask(void);

#if 1
static void Display_Description(void);
static void Display_FinalDescription(int mode);
static void Create_VUMeter(int leftLevel, int rightLevel);
#endif

int gUSBInterface = 0;

extern int gUSBrunning;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
  
  /* Configure the System clock to have a frequency of 216 MHz */
  SystemClock_Config();
  
  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  
#if 0
  /*
   * Disable LCD and back light
   */
  {
	  GPIO_InitTypeDef gpio_init_structure;
	  __HAL_RCC_GPIOI_CLK_ENABLE();
	  __HAL_RCC_GPIOK_CLK_ENABLE();

	  gpio_init_structure.Pull      = GPIO_NOPULL;
	  gpio_init_structure.Speed     = GPIO_SPEED_FAST;

	  /* LCD_DISP GPIO configuration */
	  gpio_init_structure.Pin       = GPIO_PIN_12;     /* LCD_DISP pin has to be manually controlled */
	  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
	  HAL_GPIO_Init(GPIOI, &gpio_init_structure);

	  /* LCD_BL_CTRL GPIO configuration */
	  gpio_init_structure.Pin       = GPIO_PIN_3;  /* LCD_BL_CTRL pin has to be manually controlled */
	  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
	  HAL_GPIO_Init(GPIOK, &gpio_init_structure);

	  /* Assert display enable LCD_DISP pin */
	  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);

	  /* Assert backlight LCD_BL_CTRL pin */
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_RESET);
  }
#endif
  
  /***********************************************************/
#if 0
  //just for one of the GUI demos
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIM3;

  /* Initialize TIM3 peripheral as follows:
       + Period = 500 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = 500 - 1;
  TimHandle.Init.Prescaler = uwPrescalerValue;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    while(1)
    {
    }
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    while(1)
    {
    }
  }
#endif

  /***********************************************************/

#if 0
  //enable SDRAM - seems to be needed for GUI
  /* Init the STemWin GUI Library */
  BSP_SDRAM_Init(); /* Initializes the SDRAM device */
  __HAL_RCC_CRC_CLK_ENABLE(); /* Enable the CRC Module */
#endif

#if 0
  //just to try the GUI
  GUI_Init();

  GUI_DispStringAt("Starting...", 0, 0);

  GUI_Initialized = 1;

  /* Activate the use of memory device feature */
  WM_SetCreateFlags(WM_CF_MEMDEV);

  MainTask();
#endif

  /***********************************************************/
#if 1
  {
	  //uint8_t  lcd_status = LCD_OK;

	  /* Configure the User Button in GPIO Mode */
	  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

	  /*##-1- Initialize the LCD #################################################*/
	  /* Initialize the LCD */
	  /* lcd_status = */ BSP_LCD_Init();
	  //ASSERT(lcd_status != LCD_OK);

	  /* Initialize the LCD Layers */
	  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);

	  do
	  {
		  int ifSelect;

		  //necessary to call for init of display - one instruction there
		  Display_Description();

		  //wait for button press
		  //while(CheckForUserInput() == 0) ;

		  ifSelect = Touchscreen_demo();

		  Display_FinalDescription(ifSelect);

		  switch (ifSelect)
		  {
		  	  case 1 : SPDIF_Interface(0);
		  	  	  	   break;
		  	  case 2 : SPDIF_Interface(1);
		  	  	  	   break;
		  	  case 4 : USB_Interface(0);
		  	  	  	   break;
#if 1
		  	  case 8 : MIC_Interface(0);
		  	  	  	   break;
#endif
		  	  default : //keep looping
		  		  	   break;
		  }

	  } while(1);
  }
#endif


  /* Run Application (Interrupt mode) */
  while (1)
  {
    Toggle_Leds();
  }
}

static void SPDIF_Interface(int mode)
{
	  uint32_t tickstart = 0;
	  uint32_t vuLevel;
	  uint8_t fVULevelL, fVULevelR;

	  /*
	   * ATT: do it in this order !
	   */

	  gUSBInterface = 0;

	  /* Init SPDIFRX */
	  BSP_SPDIFRX_Init();

	  if (mode == 0)
		  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE /*OUTPUT_DEVICE_AUTO*/, 90, 48000);
	  else
		  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE /*OUTPUT_DEVICE_AUTO*/, 90, 96000);

	  /* Update the Audio frame slot configuration to match the PCM standard instead of TDM */
	  BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

	  tickstart = HAL_GetTick();
	  fVULevelL = fVULevelR = 0;
	  while (1)
	  {
		SPDIFRX_Loop();

		if (Touchscreen_CheckTouchForReset())
		{
			*((uint32_t *)0xE000ED0C) = 0x05FA0004;		//soft reset entire system
		}

		if((HAL_GetTick() - tickstart) > 100)
		{
			tickstart = HAL_GetTick();
			vuLevel = HAL_SAI_GetVULevels();
			if (fVULevelL < (vuLevel >> (20-1) & 0xF))
				fVULevelL = (vuLevel >> (20-1) & 0xF);
			else
				if (fVULevelL > 0)
					fVULevelL--;

			if (fVULevelR < (vuLevel >> (4-1) & 0xF))
				fVULevelR = (vuLevel >> (4-1) & 0xF);
			else
				if (fVULevelR > 0)
					fVULevelR--;
			Create_VUMeter(fVULevelL, fVULevelR);
		}
	  }
}

#ifdef USB_AUDIO
static void USB_Interface(int mode)
{
	  uint32_t tickstart = 0;
	  uint32_t vuLevel;
	  uint8_t fVULevelL, fVULevelR;

	  gUSBInterface = 1;

#if 0
	  //it does not work (yet) for 96KHz
	  USBD_AUDIO_Set(mode);
#endif

	  /* Init Device Library */
	  USBD_Init(&USBD_Device, &AUDIO_Desc, 0);

	  /* Add Supported Class */
	  USBD_RegisterClass(&USBD_Device, USBD_AUDIO_CLASS);

	  /* Add Interface call backs for AUDIO Class */
	  USBD_AUDIO_RegisterInterface(&USBD_Device, &USBD_AUDIO_fops);

	  /* Start Device Process */
	  USBD_Start(&USBD_Device);

	  tickstart = HAL_GetTick();
	  fVULevelL = fVULevelR = 0;

	  while(1)
	  {
#if 1
		  //bug fix: do Touch Screen check just if we have really received already via USB
		  //otherwise we crash here, of calling too early: USB Audio system seems to enable something quite late
		  //root cause not yet found - but this seems to work: just: if no USB audio received - touch screen is not active!
		  if (gUSBrunning)		//this is set by first USB half buffer reception, in usbd_audio.c
		  {
			  if (Touchscreen_CheckTouchForReset())
			  //but user button works - USB does disable the touch ????
			  //so, we had to use reset or user button to go back to interface select
#else
			  if (CheckForUserInput())
#endif
			  {
				  //do a system soft reset - reset all and start over
				  *((uint32_t *)0xE000ED0C) = 0x05FA0004;		//soft reset entire system
			  }
		  }

		  if((HAL_GetTick() - tickstart) > 100)
		  {
			  tickstart = HAL_GetTick();
			  vuLevel = HAL_SAI_GetVULevels();
			  if (fVULevelL < (vuLevel >> (20-1) & 0xF))
					fVULevelL = (vuLevel >> (20-1) & 0xF);
			  else
					if (fVULevelL > 0)
						fVULevelL--;

			  if (fVULevelR < (vuLevel >> (4-1) & 0xF))
					fVULevelR = (vuLevel >> (4-1) & 0xF);
			  else
					if (fVULevelR > 0)
						fVULevelR--;
			  Create_VUMeter(fVULevelL, fVULevelR);
		  }
	  }
}
#endif

#if 1
static void MIC_Interface(int mode)
{
#define MIC_SAMPLES	128

	  uint32_t tickstart = 0;
	  uint32_t vuLevel;
	  uint8_t fVULevelL, fVULevelR;

	  uint16_t *audioBuffer;

	  /*
	   * ATT: do it in this order !
	   */

	  gUSBInterface = 0;

	  audioBuffer = (uint16_t *)malloc(MIC_SAMPLES * 2);

	  /* Init OnboardMIC */
	  BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2, OUTPUT_DEVICE_HEADPHONE, 100, DEFAULT_AUDIO_IN_FREQ);

	  BSP_AUDIO_IN_Record(audioBuffer, MIC_SAMPLES*2);

	  /* Update the Audio frame slot configuration to match the PCM standard instead of TDM */
	  BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

	  /* ATT: size to play in bytes ! */
	  BSP_AUDIO_OUT_Play(audioBuffer, MIC_SAMPLES*2 * sizeof(uint16_t));

	  tickstart = HAL_GetTick();
	  fVULevelL = fVULevelR = 0;
	  while (1)
	  {
		if (Touchscreen_CheckTouchForReset())
		{
			*((uint32_t *)0xE000ED0C) = 0x05FA0004;		//soft reset entire system
		}

		if((HAL_GetTick() - tickstart) > 100)
		{
			tickstart = HAL_GetTick();
			vuLevel = HAL_SAI_GetVULevels();
			if (fVULevelL < (vuLevel >> (20-1) & 0xF))
				fVULevelL = (vuLevel >> (20-1) & 0xF);
			else
				if (fVULevelL > 0)
					fVULevelL--;

			if (fVULevelR < (vuLevel >> (4-1) & 0xF))
				fVULevelR = (vuLevel >> (4-1) & 0xF);
			else
				if (fVULevelR > 0)
					fVULevelR--;
			Create_VUMeter(fVULevelL, fVULevelR);
		}
	  }
}
#endif

/**
  * @brief  Clock Config.
  * @param  hsai: might be required to set audio peripheral predivider if any.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
  *         Being __weak it can be overwritten by the application     
  * @retval None
  */
void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq, void *Params)
{
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  /* Set the PLL configuration according to the audio frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLSAI prescalers */
    /* PLLI2S_VCO: VCO_429M
    SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
    SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 214.5/19 = 11.289 Mhz */
#ifdef ONBOARD_DAC
    RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 429;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 2;
    RCC_ExCLKInitStruct.PLLI2SDivQ = 19;
#else
    RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 429;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 2;
    RCC_ExCLKInitStruct.PLLI2SDivQ = 19;
#endif
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K), AUDIO_FREQUENCY_96K */
  {
    /* SAI clock config
    PLLI2S_VCO: VCO_344M
    SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 344/7 = 49.142 Mhz
    SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 49.142/1 = 49.142 Mhz */
#ifdef ONBOARD_DAC
    RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 344;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 7;
    RCC_ExCLKInitStruct.PLLI2SDivQ = 1;
#else
    RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 344;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 7;
    RCC_ExCLKInitStruct.PLLI2SDivQ = 1;
#endif
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLLSAI_N                       = 384
  *            PLLSAI_P                       = 8
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Toggle LEDs to show user input state.   
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  static uint32_t ticks;
  
  if(ticks++ == 0xfffff)
  {
    BSP_LED_Toggle(LED1);
    ticks = 0;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#if 1
/**
  * @brief EXTI line detection callbacks.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t debounce_time = 0;

  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    /* Prevent debounce effect for user key */
    if ((HAL_GetTick() - debounce_time) > 50)
    {
      debounce_time = HAL_GetTick();
    }
  }
  else if (GPIO_Pin == AUDIO_IN_INT_GPIO_PIN)
  {
    /* Audio IN interrupt */
  }
}
#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BSP_Background();
}

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this application:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __HAL_RCC_TIM3_CLK_ENABLE();

  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
* @brief  BSP_Background.
* @param  None
* @retval None
*/
void BSP_Background(void)
{
  BSP_LED_Toggle(LED1);
}

#if 1
/**
  * @brief  Display main demo messages.
  * @param  None
  * @retval None
  */
static void Display_Description(void)
{
  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

#if 0
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F746G Audio", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Interface Selection", CENTER_MODE);

#if 1
  /* Draw Bitmap */
  BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 64) / 2, 62, (uint8_t *)logo);
#endif

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t *)"Copyright (c) \"Lyrebird True24\" DAC", CENTER_MODE);

  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, BSP_LCD_GetYSize() / 2 + 15, BSP_LCD_GetXSize(), 60);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 30, (uint8_t *)"Press blue User Button to start", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 45, (uint8_t *)"selecting the audio interface", CENTER_MODE);
#endif
}

static void Display_FinalDescription(int mode)
{
  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F746G Audio", CENTER_MODE);
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  switch (mode)
  {
  	  case 0 : //fall through
  	  case 1 : BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"SPDIF 48KHz", CENTER_MODE);
  	  	  	   break;
  	  case 2 : BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"SPDIF 96KHz", CENTER_MODE);
  	  	  	   break;
  	  case 4 : BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"USB 48KHz", CENTER_MODE);
  	  	  	   break;
  	  case 8 : BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Onboard MIC", CENTER_MODE);
  	  	  	   break;
  }

  /* Draw Bitmap */
#if 1
  BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 64) / 2, 62, (uint8_t *)logo);
#endif

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t *)"Copyright (c) \"Lyrebird True24\" DAC", CENTER_MODE);

  BSP_LCD_SetFont(&Font16);
  //BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  //BSP_LCD_FillRect(0, BSP_LCD_GetYSize() / 2 + 15, BSP_LCD_GetXSize(), 60);
  //BSP_LCD_FillRect(0, BSP_LCD_GetYSize() / 2 + 15, BSP_LCD_GetXSize(), 45);
  //BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  //BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 30, (uint8_t *)"touch Lyrebird icon or press reset", CENTER_MODE);
  //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 45, (uint8_t *)"in order to select new interface", CENTER_MODE);

  Create_VUMeter(12, 16);
}

static void Create_VUMeter(int leftLevel, int rightLevel)
{
#define X_WIDTH	25
#define Y_WIDTH	20
#define XY_GAP   2

	int i;

	for (i = 0; i < 16; i++)
	{
		if (leftLevel > i)
		{
			if (i > 13)
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
			else
			if (i > 10)
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
			else
				BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		}
		else
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
		BSP_LCD_FillRect(22 + i*(X_WIDTH + XY_GAP), BSP_LCD_GetYSize() / 2 + 15 + 50, X_WIDTH, Y_WIDTH);

		if (rightLevel > i)
		{
			if (i > 13)
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
			else
			if (i > 10)
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
			else
				BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			}
			else
				BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
		BSP_LCD_FillRect(22 + i*(X_WIDTH + XY_GAP), BSP_LCD_GetYSize() / 2 + 15 + 50 + Y_WIDTH + XY_GAP, X_WIDTH, Y_WIDTH);
	}
}
#endif

/**
  * @brief  Check for user input.
  * @param  None
  * @retval Input state (1 : active / 0 : Inactive)
  */
uint8_t CheckForUserInput(void)
{
  if (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    HAL_Delay(10);
    while (BSP_PB_GetState(BUTTON_KEY) != RESET);
    return 1 ;
  }
  return 0;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
