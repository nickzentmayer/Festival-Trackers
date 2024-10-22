/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "bdma.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include "lsm303agr.h"
#include "gps_tools.h"
#include "max17048.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "st7701.h"
#include "graphics.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GGA_BUFFER_SIZE 100
#define POWER_ON_HOLD_MS 3000
#define POWER_OFF_HOLD_MS 3000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//uint16_t framebuffer[480][480];  //16 bpp framebuffer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DISPLAY_WIDTH        480
#define DISPLAY_HEIGHT       480
#define COLOR_BLUE           0x001F  // 16-bit RGB color: Blue

void display_blue(void) {
    // Fill the screen with a solid color
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            framebuffer[x][y] = COLOR_BLUE;
        }
    }
}

void DMA2D_FillRect(uint32_t color, uint32_t width, uint32_t height)
{
  DMA2D_HandleTypeDef hdma2d;
  hdma2d.Instance = DMA2D;

  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = DISPLAY_WIDTH - width;

  HAL_DMA2D_Init(&hdma2d);
  HAL_DMA2D_Start(
    &hdma2d,
    color,
	(uint32_t) framebuffer,
    width,
    height);
  HAL_DMA2D_PollForTransfer(&hdma2d, 10);
}

int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*)ptr, len);  // Transmit data via USB CDC
    return len;
}

LSM303AGR_AccelData accel_data;
LSM303AGR_MagData mag_data;

uint8_t rx_buffer[1];     // Buffer for receiving data via interrupt
char gga_buffer[GGA_BUFFER_SIZE]; // Larger buffer to store the accumulated data
uint8_t gga_index = 0;  // Index to keep track of the current position in data_buffer

GPS_Data gps_data;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_LTDC_Init();
  MX_I2C4_Init();
  MX_TIM1_Init();
  MX_UART8_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  MX_DMA2D_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init(); // Initialize USB CDC

  // power latch section. change delay to adjust power-on hold time
  HAL_Delay(POWER_ON_HOLD_MS);
  HAL_GPIO_WritePin(GPIOC, LDO_EN_Pin, GPIO_PIN_SET); // keep the LDO_EN set high when BTN released

  // update period of TIM6 to configure power-off timer
  __HAL_TIM_SET_AUTORELOAD(&htim6, POWER_OFF_HOLD_MS);

  // do extra LTDC initialization stuff

  // adjust the LTDC framebuffer address to the new private variable `framebuffer`
  HAL_LTDC_SetAddress(&hltdc, (uint32_t)&framebuffer, LTDC_LAYER_1);

  HAL_StatusTypeDef st7701_init_status = ST7701_Init();
	if (st7701_init_status != HAL_OK) {
	  printf("st7701 initialization failed!\r\n");
	} else {
	  printf("st7701 initialized!\r\n");
    }
	fflush(stdout);


  // start receiving data on UART8 via interrupt, for gps
  HAL_UART_Receive_IT(&huart8, (uint8_t*)rx_buffer, 1);

  // this one is pretty self-explanatory
  LSM303AGR_Init(&hi2c4);

//  HAL_GPIO_WritePin(GPIOB, GPS_ON_Pin, GPIO_PIN_SET);

//   Start PWM output on TIM1 Channel 1
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
	  // Initialization Error
	  Error_Handler();
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500); // half brightness

  // -------------- TESTING GRAPHICS LIBRARY -----------------
  Layer layer = { .head = NULL, .tail = NULL };

  Line *line = createLine(100, 250, 300, 100, rgbToColor(255, 50, 0));
  Line *line2 = createLine(300, 100, 100, 250, rgbToColor(50, 255, 0));
  Rectangle *rect = createRectangle(100, 100, 200, 150, rgbToColor(150, 0, 255));

  addDrawable(&layer, (Drawable *)rect);
  addDrawable(&layer, (Drawable *)line);
  addDrawable(&layer, (Drawable *)line2);
  int startTick = 0;
  int endTick = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  startTick = HAL_GetTick();
	  DMA2D_FillRect(0x00000000, DISPLAY_WIDTH, DISPLAY_HEIGHT);

	  drawLayerAlways(&layer);


	  endTick = HAL_GetTick();
	  if (endTick-startTick < 16) HAL_Delay(16-endTick-startTick);
	  printf("%d ms\r\n", endTick-startTick);
	  fflush(stdout);
  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 76;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV8;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// called when BTN is pressed
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	printf("BTN PRESSED");
	fflush(stdout);
    if (GPIO_Pin == GPIO_PIN_15) { // Check if the interrupt is from PC15
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET) {
            // Button is pressed: Start 3-second timer
        	printf("BTN STILL PRESSED\n");
			fflush(stdout);
            __HAL_TIM_SET_COUNTER(&htim6, 0); // Reset the timer counter
            HAL_TIM_Base_Start_IT(&htim6);
            printf("TIMER STARTED\n");
            fflush(stdout);
        } else {
            // Button is released: Stop the timer
            HAL_TIM_Base_Stop_IT(&htim6);
        }
    }
}

// triggered at the end of each frame
void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc)
{
    printf("frame done!\r\n");
    fflush(stdout);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (rx_buffer[0] == '$') {
		// Start of a new GGA sentence, reset the index
		gga_index = 0;
	} else if (rx_buffer[0] == '\r') {
		// End of GGA sentence

//		if (gga_index < GGA_BUFFER_SIZE) {
//			gga_buffer[gga_index] = '\0'; // Null-terminate for string processing
//		}

		// Process the complete GGA sentence
		bool success = Process_GGA_Sentence(gga_buffer, &gps_data);
		if (success) printf("%d satellites", gps_data.num_satellites);

		// Reset the index to start a new sentence
		gga_index = 0;
	} else if (gga_index < GGA_BUFFER_SIZE) {
		// accumulate the received character if we haven't reached the buffer size
		gga_buffer[gga_index++] = rx_buffer[0];
	} else {
		// there's something wrong, reset the index to 0 to start the data buffer over
		gga_index = 0;
	}

	// re-enable the interrupt to receive the next byte
	HAL_UART_Receive_IT(huart, rx_buffer, 1);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM6) { // Check if this is the interrupt from TIM6
		printf("TIMER FINISHED");
		fflush(stdout);
		// Code to execute when the timer reaches the period (e.g., every 3 seconds)
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // Set PC1 low
		}

		// Stop the timer to prevent continuous triggering (optional)
		HAL_TIM_Base_Stop_IT(htim);
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
