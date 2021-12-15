#include "main.h"
#include <traj_gen.h>

#define MOTOR_STEP_ANGLE 1.8f // deg
#define MICROSTEPS 8
#define DEGREES_PER_STEP (MOTOR_STEP_ANGLE / MICROSTEPS)
#define CLOCK_FREQ 72000000                       // Hz
#define STEPPER_TIMER_FREQ 1000000                // Hz
#define MIN_STEP_PERIOD 80                        // us
#define ANG_ACCEL 500000                          // deg/s^2
#define STEP_ACCEL (ANG_ACCEL * MOTOR_STEP_ANGLE) // stp/s^2

// speed control params
static Movement_FSMTypeDef mode;
static float count;     // init for the count counter
static float lastcount; // c_i-1
static float nextcount; // c_i
static int32_t step;    // the current step we are on (from 0)
static int32_t n_step;  // this one is decremented for deceleration
static int32_t goalsteps;
static Trapezoidal_MoveTypeDef current_move;

int stepcommands[10];

int error = 0;

static TIM_HandleTypeDef htim2 = {
    .Instance = TIM2};

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (mode != STOP)
  {
    count++;
    if (count >= nextcount)
    {
      // do a step
      HAL_GPIO_WritePin(X_STEP_PORT, X_STEP_PIN, GPIO_PIN_SET);
      // at 72Mhz 8 NOPs= 112ns > 100ns min step.
      // okay it even works with none... is HAL GPIO really that slow? I wish I could find my oscilloscope.
      // for (size_t i = 0; i < 8; i++)
      // {
      //   __ASM volatile("NOP");
      // }
      HAL_GPIO_WritePin(X_STEP_PORT, X_STEP_PIN, GPIO_PIN_RESET);

      step++;

      // change our goal if necessary
      if (mode == ACCEL)
      {
        if (step >= goalsteps)
        {
          goalsteps = current_move.decel_steps;
          mode = RUN;
        }

        n_step++;

        // calculate when to step next
        // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
        nextcount = nextcount - ((2 * nextcount) / (4 * n_step + 1));
      }
      else if (mode == RUN)
      {
        if (step >= goalsteps)
        {
          mode = DECEL;
          goalsteps = current_move.steps;
        }

        // use the same count as last step
        nextcount = count;
      }
      else if (mode == DECEL)
      {
        if (step >= goalsteps)
        {
          mode = STOP;
        }

        n_step--;
        nextcount = nextcount - ((2 * nextcount) / (4 * -n_step + 1));
      }

      count = 0;
    }
  }
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
  htim2.Init.Prescaler = 72 - 1; // 1 us period
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.RepetitionCounter = 0;
  // htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    error = 1;
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
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

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  TIM2_Init();

  Speed_Profile_ParamsTypeDef profile_params = {
      .acceleration = ANG_ACCEL,
      .deceleration = ANG_ACCEL,
      .current_speed = 0,
      .final_speed = 0,
      .max_speed = 15000,
      .steps = 10 * 360 / DEGREES_PER_STEP,
      .counter_freq = STEPPER_TIMER_FREQ,
      .degreesperstep = DEGREES_PER_STEP,
  };

  generate_trap_profile(profile_params, &current_move);

  goalsteps = current_move.accel_steps;
  lastcount = current_move.starting_count;
  count = 0;
  nextcount = lastcount;
  step = 1;
  n_step = 1;

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    error = 1;
  }

  HAL_Delay(1000);
  mode = ACCEL; // this should probably be more generic, then let the planner figure it out.

  while (1)
  {
  }
}