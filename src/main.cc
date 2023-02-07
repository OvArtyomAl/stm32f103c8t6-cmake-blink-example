extern "C" {
  #include "stm32f1xx.h"
}

void SystemClockConfig();
void InitLED();
void ToggleLED();
void ErrorHandler();

int main() {
  HAL_Init();

  SystemClockConfig();
  InitLED();

  while(1) {
    ToggleLED();
    HAL_Delay(500);
  }

  return 1;
}

void SystemClockConfig() {
  RCC_OscInitTypeDef RCC_OscInitStruct{};
  RCC_ClkInitTypeDef RCC_ClkInitStruct{};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    ErrorHandler();
  }
}

void InitLED() {
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio_conf{};
  gpio_conf.Pin = GPIO_PIN_13;
  gpio_conf.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_conf.Pull = GPIO_NOPULL;
  gpio_conf.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio_conf);
}

void ToggleLED() {
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void ErrorHandler() {
  while(1) ;
}

extern "C" {

void SysTick_Handler(void) {
  HAL_IncTick();
}

} // extern "C"
