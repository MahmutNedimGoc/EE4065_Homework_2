/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_image.h"
#include "lib_serialimage.h"
/* USER CODE END Includes */
#define IMG_W   128
#define IMG_H   128
#define IMG_SIZE (IMG_W * IMG_H)

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t hist[256];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void ComputeHistogram_Grayscale(uint8_t *p, uint32_t hist[256])
{
    for (int i = 0; i < 256; i++)
        hist[i] = 0;

    for (uint32_t i = 0; i < IMG_SIZE; i++)
    {
        uint8_t val = p[i];
        hist[val]++;
    }
}

void HistogramEqualization(uint8_t *p)
{
    uint32_t hist[256] = {0};
    uint32_t cdf[256];
    uint8_t  lut[256];

    for (uint32_t i=0; i<IMG_SIZE; i++)
        hist[p[i]]++;

    // CDF
    uint32_t cum = 0;
    for (int i=0; i<256; i++)
    {
        cum += hist[i];
        cdf[i] = cum;
    }

    // CDF min
    uint32_t cdf_min = 0;
    for (int i = 0; i < 256; i++)
        if (cdf[i] != 0) { cdf_min = cdf[i]; break; }

    // LUT
    for (int i=0; i<256; i++)
    {
        float num = (float)(cdf[i] - cdf_min);
        float den = (float)(IMG_SIZE - cdf_min);
        float sk  = (num / den) * 255.0f;

        int val = (int)(sk + 0.5f);
        if (val < 0) val = 0;
        if (val > 255) val = 255;

        lut[i] = (uint8_t)val;
    }

    for (uint32_t i = 0; i < IMG_SIZE; i++)
        p[i] = lut[p[i]];
}
/* USER CODE BEGIN 0 */
uint8_t img_low[IMG_SIZE];
uint8_t img_high[IMG_SIZE];
uint32_t hist_before[256];
uint32_t hist_after[256];
uint8_t img_median[IMG_SIZE];




// 3x3 low-pass
const int8_t kernel_low[3][3] =
{
    { 1, 1, 1 },
    { 1, 1, 1 },
    { 1, 1, 1 }
};

// 3x3 high-pass
const int8_t kernel_high[3][3] =
{
    {  0, -1,  0 },
    { -1,  4, -1 },
    {  0, -1,  0 }
};

// 3x3 convolve
void Convolve3x3(const uint8_t *in,uint8_t *out,const int8_t kernel[3][3],int divisor,int offset)
{
    for (int y = 0; y < IMG_H; y++)
    {
        for (int x = 0; x < IMG_W; x++)
        {

            if (y == 0 || y == IMG_H - 1 || x == 0 || x == IMG_W - 1)
            {
                out[y * IMG_W + x] = 0;
            }
            else
            {
                int acc = 0;

                acc += in[(y-1) * IMG_W + (x-1)] * kernel[0][0];
                acc += in[(y-1) * IMG_W + (x  )] * kernel[0][1];
                acc += in[(y-1) * IMG_W + (x+1)] * kernel[0][2];

                acc += in[(y  ) * IMG_W + (x-1)] * kernel[1][0];
                acc += in[(y  ) * IMG_W + (x  )] * kernel[1][1];
                acc += in[(y  ) * IMG_W + (x+1)] * kernel[1][2];

                acc += in[(y+1) * IMG_W + (x-1)] * kernel[2][0];
                acc += in[(y+1) * IMG_W + (x  )] * kernel[2][1];
                acc += in[(y+1) * IMG_W + (x+1)] * kernel[2][2];


                if (divisor != 0)
                {
                    if (acc >= 0)
                        acc = (acc + divisor/2) / divisor;   // pozitif için yuvarlama
                    else
                        acc = (acc - divisor/2) / divisor;   // negatif için yuvarlama
                }

                // offset
                acc += offset;


                if (acc < 0)   acc = 0;
                if (acc > 255) acc = 255;

                out[y * IMG_W + x] = (uint8_t)acc;
            }
        }
    }
}
// Q4 - 3×3 Median Filter
void MedianFilter3x3(const uint8_t *in, uint8_t *out)
{
    uint8_t window[9];

    for (int y = 0; y < IMG_H; y++)
    {
        for (int x = 0; x < IMG_W; x++)
        {

            if (y == 0 || x == 0 || y == IMG_H-1 || x == IMG_W-1)
            {
                out[y * IMG_W + x] = 0;
                continue;
            }


            int k = 0;
            for (int j = -1; j <= 1; j++)
            {
                for (int i = -1; i <= 1; i++)
                {
                    window[k++] = in[(y + j) * IMG_W + (x + i)];
                }
            }


            for (int a = 0; a < 9-1; a++)
            {
                for (int b = 0; b < 9-a-1; b++)
                {
                    if (window[b] > window[b+1])
                    {
                        uint8_t tmp = window[b];
                        window[b] = window[b+1];
                        window[b+1] = tmp;
                    }
                }
            }


            out[y * IMG_W + x] = window[4];
        }
    }
}

/* USER CODE END 0 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 128x128 çözünürlük, RGB565 (2 byte) formatı için buffer
volatile uint8_t pImage[128*128*1];
IMAGE_HandleTypeDef img;
/* USER CODE END 0 */



int main(void)
{


  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();



  LIB_IMAGE_InitStruct(&img, (uint8_t*)pImage, 128, 128, IMAGE_FORMAT_GRAYSCALE);



  while (LIB_SERIAL_IMG_Receive(&img) != SERIAL_OK){
  		HAL_Delay(100);
    }


  uint8_t *p = img.pData;
  //Q1
  ComputeHistogram_Grayscale(&img, hist_before);
  HAL_Delay(2000);
  //Q2
  HistogramEqualization(p);
  HAL_Delay(2000);
  ComputeHistogram_Grayscale(p, hist_after);
  img.pData = p;
  LIB_SERIAL_IMG_Transmit(&img);
  HAL_Delay(2000);
  //Q3
  Convolve3x3((uint8_t*)pImage, img_low, kernel_low, 9, 0);
  img.pData = img_low;
  LIB_SERIAL_IMG_Transmit(&img);
  HAL_Delay(2000);

  Convolve3x3((uint8_t*)pImage, img_high, kernel_high, 1, 128);
  img.pData = img_high;
  LIB_SERIAL_IMG_Transmit(&img);
  HAL_Delay(2000);

  MedianFilter3x3(pImage, img_median);
  img.pData = img_median;
  LIB_SERIAL_IMG_Transmit(&img);

  while(1){

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 2000000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
