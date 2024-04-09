/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define MIC_SAMPLE_FREQUENCY 8000
#define MIC_SAMPLES_PER_MS (MIC_SAMPLE_FREQUENCY/1000)  // ==8 //== 48
#define MIC_NUM_CHANNELS 1
#define MIC_MS_PER_PACKET 120//120    //20
#define MIC_SAMPLES_PER_PACKET (MIC_SAMPLES_PER_MS * MIC_MS_PER_PACKET) // ==160 //== 960

// 20ms of 64 bit samples
// allocate buffers
volatile int32_t _sampleBuffer[MIC_SAMPLES_PER_PACKET * 2];// 7680 bytes (*2 because samples are 64 bit)
int16_t _processBuffer[MIC_SAMPLES_PER_PACKET];            // 1920 bytes
int16_t _sendBuffer[MIC_SAMPLES_PER_PACKET];               // 1920 bytes

bool _running = false;






void ei_printf(const char *format, ...);
void vprint(const char *fmt, va_list argp);



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Constructor
 */
void Audio_init() {
  // initialise variables

  // set LR to low (it's pulled low anyway)
  //HAL_GPIO_WritePin(LR_GPIO_Port, LR_Pin, GPIO_PIN_RESET);
}

/**
 * Start the I2S DMA transfer (called from usbd_audio_if.cpp)
 */
int8_t Audio_start() {

  HAL_StatusTypeDef status;

  // HAL_I2S_Receive_DMA will multiply the size by 2 because the standard is 24 bit Philips.

  if ((status = HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) _sampleBuffer, MIC_SAMPLES_PER_PACKET)) == HAL_OK) {
    _running = true;
  }

  return status;
}

/**
 * Stop the I2S DMA transfer
 */
int8_t Audio_stop() {

  HAL_StatusTypeDef status;

  if ((status = HAL_I2S_DMAStop(&hi2s1)) == HAL_OK) {
    _running = false;
  }
  return status;
}

/**
 * Pause I2S DMA transfer (soft-mute)
 */
int8_t Audio_pause() {

  HAL_StatusTypeDef status;

  if ((status = HAL_I2S_DMAPause(&hi2s1)) == HAL_OK) {
    _running = false;
  }

  return status;
}

/**
 * Resume I2S DMA transfer
 */
int8_t Audio_resume() {

  HAL_StatusTypeDef status;

  if ((status = HAL_I2S_DMAResume(&hi2s1)) == HAL_OK) {
    _running = true;
  }
  return status;
}

/**
 * 1. Transform the I2S data into 16 bit PCM samples in a holding buffer
 * 2. Use the ST GREQ library to apply a graphic equaliser filter
 * 3. Use the ST SVC library to adjust the gain (volume)
 * 4. Transmit over USB to the host
 *
 * We've got 10ms to complete this method before the next DMA transfer will be ready.
 */
void sendData(volatile int32_t *data_in, int16_t *data_out) {

  // only do anything at all if we're connected

  if (_running) {

    if (0) {

      // we're muted so zero this half of the send buffer. this works around Windows/Citrix habit
      // of disconnecting the device if the stream of sample data suddenly pauses.
			for (uint16_t i = 0; i < MIC_SAMPLES_PER_PACKET / 2; i++) {
				data_out[i] = 0;
			}
			
    } else {

      // transform the I2S samples from the 64 bit L/R (32 bits per side) of which we
      // only have data in the L side. Take the most significant 16 bits, being careful
      // to respect the sign bit.

      int16_t *dest = _processBuffer;

      for (uint16_t i = 0; i < MIC_SAMPLES_PER_PACKET / 2; i++) {

        // dither the LSB with a random bit

        int16_t sample = (data_in[0] & 0xfffffffe) | (rand() & 1);

        *dest++ = sample;     // left channel has data
        *dest++ = sample;     // right channel is duplicated from the left
        data_in += 2;
      }

      // apply the graphic equaliser filters using the ST GREQ library then
      // adjust the gain (volume) using the ST SVC library

      //_graphicEqualiser.process(_processBuffer, MIC_SAMPLES_PER_PACKET / 2);
      //_volumeControl.process(_processBuffer, MIC_SAMPLES_PER_PACKET / 2);

      // we only want the left channel from the processed buffer

      int16_t *src = _processBuffer;
      dest = data_out;

      for (uint16_t i = 0; i < MIC_SAMPLES_PER_PACKET / 2; i++) {
        *dest++ = *src;
        src += 2;
      }
    }

    // send the adjusted data to the host

//    if (USBD_AUDIO_Data_Transfer(&hUsbDeviceFS, data_out, MIC_SAMPLES_PER_PACKET / 2) != USBD_OK) {
//      Error_Handler();
//    }
		
#if 0		
		//FIXME: if printf time is too long, it will block the main thread, not good
		for (uint16_t i = 0; i < MIC_SAMPLES_PER_PACKET / 2; ++i) {
			ei_printf("%d,%d,%d\r\n", 256 * 4, -256 * 4, data_out[i]);
		}
#else
	int maxValue = 0;
		for (uint32_t i = 0; i < MIC_SAMPLES_PER_PACKET / 2; i++) {
    int16_t value = data_out[i];
		if (fabs((double)value) > fabs((double)maxValue)) {
			maxValue = value;
		}
		//ei_printf("%d,%d,%d\r\n", 256*128, -256*128, value);
	}
	ei_printf("%d,%d,%d\r\n", 256 * 8, -256 * 8, maxValue);	
#endif
  }
}

/**
 * Override the I2S DMA half-complete HAL callback to process the first MIC_MS_PER_PACKET/2 milliseconds
 * of the data while the DMA device continues to run onward to fill the second half of the buffer.
 */
void i2s_halfComplete() {
  sendData(_sampleBuffer, _sendBuffer);
}

/**
 * Override the I2S DMA complete HAL callback to process the second MIC_MS_PER_PACKET/2 milliseconds
 * of the data while the DMA in circular mode wraps back to the start of the buffer
 */
void i2s_complete() {
  sendData(&_sampleBuffer[MIC_SAMPLES_PER_PACKET], &_sendBuffer[MIC_SAMPLES_PER_PACKET / 2]);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  i2s_halfComplete();
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  i2s_complete();
}






/**
 * @brief  Start audio recording engine
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */
int8_t Record() {
  return Audio_start();
}

/**
 * @brief  Stops audio acquisition
 * @param  none
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */
int8_t Stop() {
  return Audio_stop();
}

/**
 * @brief  Pauses audio acquisition
 * @param  none
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */
int8_t Pause() {
  return Audio_pause();
}

/**
 * @brief  Resumes audio acquisition
 * @param  none
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */
int8_t Resume() {
  return Audio_resume();
}















/**
 * Low-level print function that uses UART to print status messages.
 */
void vprint(const char *fmt, va_list argp)
{
  char string[200];
  if(0 < vsprintf(string, fmt, argp)) // build string
  {
		//!!!!!FIXME: don't use 0xffffff or HAL_MAX_DELAY as timeout, it will block the main thread!
		//FIXME: HAL_MAX_DELAY may be good
      HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), HAL_MAX_DELAY/*100*/);
  }
}

/**
 * Wrapper for vprint. Use this like you would printf to print messages to the serial console.
 */
void ei_printf(const char *format, ...)
{
  va_list myargs;
  va_start(myargs, format);
  vprint(format, myargs);
  va_end(myargs);
}






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//int countWait = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	ei_printf("starting record...\r\n");
	Record();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		//ei_printf("countWait == %d\r\n", countWait++);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
	//Stop();
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
  PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLSRC;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLLSRC;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
