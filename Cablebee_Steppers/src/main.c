#include "main.h"

static TIM_HandleTypeDef htim2 = {
    .Instance = TIM2};
uint16_t step;
uint16_t stepduration;
uint16_t stepperiod;

int error = 0;

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (step == stepperiod)
  {
    HAL_GPIO_WritePin(X_STEP_PORT, X_STEP_PIN, GPIO_PIN_SET);
  }
  else if (step > stepperiod + stepduration)
  {
    HAL_GPIO_WritePin(X_STEP_PORT, X_STEP_PIN, GPIO_PIN_RESET);
    step = 0;
  }

  step++;
}

void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  // __HAL_RCC_PWR_CLK_ENABLE();
  /* Configure PLLs------------------------------------------------------*/

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  // sysclock 72 MHz, apb1 36Mhz
  oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
  { /* Initialization Error */
    while (1)
      ;
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
*/
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
                             RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
  { /* Initialization Error */
    while (1)
      ;
  }
}

void TIM2_Init(void)
{
  htim2.Init.Prescaler = 7 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.RepetitionCounter = 0;
  // htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    error = 1;
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void GPIO_Init()
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = X_ENABLE_PIN | X_DIR_PIN | X_STEP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(X_ENABLE_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(X_ENABLE_PORT, X_ENABLE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(X_DIR_PORT, X_DIR_PIN, GPIO_PIN_RESET);
}

uint16_t cnt;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  TIM2_Init();

  step = 0;
  stepduration = 2; // 500 ns
  stepperiod = 50;  // 1ms

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    error = 1;
  }

  while (1)
  {
  }
}