#include "main.h"

int main(void)
{
  HAL_Init();

  // set up pins
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = X_ENABLE_PIN | X_DIR_PIN | X_STEP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(X_ENABLE_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(X_ENABLE_PORT, X_ENABLE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(X_DIR_PORT, X_DIR_PIN, GPIO_PIN_SET);

  while (1)
  {
    HAL_GPIO_WritePin(X_STEP_PORT, X_STEP_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(X_STEP_PORT, X_STEP_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}