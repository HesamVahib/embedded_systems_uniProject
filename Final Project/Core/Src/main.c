/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Basic final assignment – PI + plant + PWM + UART + modes
  *
  * - TIM3 @ 1 kHz interrupt runs: PI -> plant -> PWM
  * - TIM2 CH1 outputs PWM duty (0..1)
  * - UART2 CLI: help, status, mode idle|mod|cfg, kp x, ki x, ref x
  * - PC13 button cycles modes (unless locked in CONFIG)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { MODE_CONFIG=0, MODE_IDLE=1, MODE_MOD=2 } mode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TS_SEC        (0.001f)   /* 1 ms */
#define UART_ECHO     1
#define Y_FULL_SCALE  (10.0f)     /* IMPORTANT: scale plant output to 0..1 */
#define STATE_LIMIT   (50.0f)
#define Y_LIMIT       (50.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile mode_t  g_mode = MODE_IDLE;
static volatile uint8_t g_uart_config_lock = 0;
static volatile uint8_t g_btn_event = 0;

/* Signals */
static volatile float g_ref  = 0.50f;   /* 0..1 */
static volatile float g_uin  = 0.00f;   /* 0..1 */
static volatile float g_uout = 0.00f;   /* plant output (raw) */
static volatile float g_meas01 = 0.0f;


/* PI */
static volatile float g_kp = 0.50f;
static volatile float g_ki = 30.0f;
static volatile float g_i  = 0.0f;
static float g_meas_f = 0.0f;


/* UART line buffer */
static char     g_line[96];
static uint32_t g_line_len = 0;

/* Plant state */
static float xplant[6] = {0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static void  uart_putc(uint8_t c);
static void  uart_print(const char *s);
static void  uart_println(const char *s);
static float clamp01(float x);
static int   finite_f(float v);

static const char* mode_str(mode_t m);

static void  reset_control(void);
static void  pwm_set(float duty01);
static float plant_step(float uin);
static float pi_step(float ref, float meas01);
static void  process_line(char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===== given plant ===== */
static const float A[6][6] = {
  { 0.9652f, -0.0172f,  0.0057f,  0.9948f,  0.2655f, -0.3848f },
  { 0.7732f,  0.1252f,  0.2315f,  0.7648f, -0.4165f, -0.4855f },
  { 0.8278f, -0.7522f, -0.0956f,  1.1056f,  0.7587f,  0.1179f },
  {-0.0058f,  0.0052f, -0.0251f,  0.4212f,  0.3927f,  0.2899f },
  { 0.0700f,  0.1282f,  0.7754f, -0.3366f, -0.0986f,  0.7281f },
  { 0.3299f, -0.4855f,  0.3915f,  0.0748f, -0.2192f,  0.1491f },
};
static const float B[6] = { 0.0471f, 0.0377f, 0.0404f, 0.0485f, 0.0373f, 0.0539f };

static int finite_f(float v)
{
  if (v != v) return 0;                 /* NaN */
  if (v > 1e30f || v < -1e30f) return 0;/* Inf-ish */
  return 1;
}

static float clamp01(float x)
{
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static void uart_putc(uint8_t c)
{
  HAL_UART_Transmit(&huart2, &c, 1, 100);
}

static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 200);
}

static void uart_println(const char *s)
{
  uart_print(s);
  uart_print("\r\n");
}

static const char* mode_str(mode_t m)
{
  switch (m) {
    case MODE_CONFIG: return "CONFIG";
    case MODE_IDLE:   return "IDLE";
    case MODE_MOD:    return "MOD";
    default:          return "?";
  }
}

static void reset_control(void)
{
  g_i = 0.0f;
  for (int k=0; k<6; k++) xplant[k] = 0.0f;
  g_uin  = 0.0f;
  g_uout = 0.0f;
}

static void pwm_set(float duty01)
{
  duty01 = clamp01(duty01);
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(duty01 * (float)arr));
}

static float plant_step(float uin)
{
  float xn[6];

  for (int r=0; r<6; r++)
  {
    float sum = 0.0f;
    for (int c=0; c<6; c++) sum += A[r][c] * xplant[c];
    sum += B[r] * uin;
    xn[r] = sum;
  }

  /* state safety */
  for (int i=0; i<6; i++)
  {
    if (!finite_f(xn[i]) || xn[i] > STATE_LIMIT || xn[i] < -STATE_LIMIT)
    {
      for (int k=0; k<6; k++) xplant[k] = 0.0f;
      return 0.0f;
    }
  }

  for (int i=0; i<6; i++) xplant[i] = xn[i];

  float y = xplant[5];
  if (!finite_f(y) || y > Y_LIMIT || y < -Y_LIMIT)
  {
    for (int k=0; k<6; k++) xplant[k] = 0.0f;
    return 0.0f;
  }

  return y;
}

/* PI with simple anti-windup */
static float pi_step(float ref, float meas01)
{
  float e  = ref - meas01;
  float up = g_kp * e;

  float ui_new = g_i + (g_ki * TS_SEC) * e;
  float u_try  = up + ui_new;

  if ((u_try < 1.0f && u_try > 0.0f) ||
      (u_try >= 1.0f && e < 0.0f) ||
      (u_try <= 0.0f && e > 0.0f))
  {
    g_i = ui_new;
  }

  float u = up + g_i;
  if (u > 1.0f) u = 1.0f;
  if (u < 0.0f) u = 0.0f;
  return u;
}

/* PC13 EXTI callback -> set flag */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13) g_btn_event = 1;
}

static void process_line(char *line)
{
  while (*line == ' ') line++;

  size_t n = strlen(line);
  while (n && (line[n-1]=='\r' || line[n-1]=='\n' || line[n-1]==' ')) line[--n] = 0;
  if (*line == '\0') return;

  if (strcmp(line, "help") == 0) {
    uart_println("help | status | mode idle|mod|cfg | kp <f> | ki <f> | ref <f>");
    return;
  }

  if (strcmp(line, "status") == 0)
  {
    __disable_irq();
    mode_t m = g_mode;
    uint8_t lock = g_uart_config_lock;
    float kp = g_kp, ki = g_ki, ref = g_ref, meas = g_meas01, uin = g_uin, uout = g_uout;
    __enable_irq();

    char buf[200];
    snprintf(buf, sizeof(buf),
      "mode=%s lock=%u kp=%.3f ki=%.3f ref=%.3f meas=%.3f uin=%.3f uout=%.3f",
      mode_str(m), (unsigned)lock,
      (double)kp, (double)ki, (double)ref, (double)meas, (double)uin, (double)uout);
    uart_println(buf);
    return;
  }
  if (strncmp(line, "mode", 4) == 0)
  {
    char *arg = line + 4;
    while (*arg == ' ') arg++;

    if (strncmp(arg, "idle", 4) == 0) {
      reset_control();
      g_mode = MODE_IDLE;
      g_uart_config_lock = 0;
      uart_println("Mode -> IDLE");
      return;
    }
    if (strncmp(arg, "mod", 3) == 0) {
      reset_control();
      g_mode = MODE_MOD;
      g_uart_config_lock = 0;
      uart_println("Mode -> MOD");
      return;
    }
    if (strncmp(arg, "cfg", 3) == 0) {
      reset_control();
      g_mode = MODE_CONFIG;
      g_uart_config_lock = 1;
      uart_println("Mode -> CONFIG");
      return;
    }

    uart_println("ERR mode arg");
    return;
  }

  /* === IMPORTANT FIX: reset PI integrator when changing params/ref === */
  if (strncmp(line, "kp", 2) == 0)  {
    g_kp = (float)atof(line + 2);
    g_i  = 0.0f;                 /* reset integrator */
    uart_println("OK kp");
    return;
  }

  if (strncmp(line, "ki", 2) == 0)  {
    g_ki = (float)atof(line + 2);
    g_i  = 0.0f;                 /* reset integrator */
    uart_println("OK ki");
    return;
  }

  if (strncmp(line, "ref", 3) == 0)  {
    g_ref = clamp01((float)atof(line + 3));
    g_i   = 0.0f;                /* reset integrator */
    uart_println("OK ref");
    return;
  }

  uart_println("ERR unknown (type help)");
}


/* TIM3 ISR: PI -> plant -> PWM */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3) return;

  if (!finite_f(g_uout) || !finite_f(g_i) ||
      !finite_f(g_kp)  || !finite_f(g_ki) || !finite_f(g_ref) ||
      g_uout > 20.0f || g_uout < -20.0f)
  {
    reset_control();
    pwm_set(0.0f);
    return;
  }

  if (g_mode == MODE_IDLE)
  {
    g_uin = 0.0f;
    pwm_set(0.0f);
    g_uout = plant_step(0.0f);
    g_meas01 = 0.0f;   /* because y clamped at 0 */
  }
  else if (g_mode == MODE_MOD)
  {
	  /* ---- measurement: magnitude + low-pass ---- */
	  float y = g_uout;
	  if (y < 0.0f) y = -y;                 // magnitude

	  /* normalize */
	  float meas01 = y / Y_FULL_SCALE;
	  meas01 = clamp01(meas01);

	  /* 1st order low-pass: tau ~ 50ms */
	  const float alpha = 0.02f;            // 0.02 @ 1kHz -> ~50ms
	  g_meas_f += alpha * (meas01 - g_meas_f);

	  g_meas01 = g_meas_f;                  // for status print
	  /* ------------------------------------------- */

	  float u = pi_step(g_ref, g_meas_f);

	  g_uin  = u;
	  g_uout = plant_step(u);
	  pwm_set(u);

  }
  else /* MODE_CONFIG */
  {
    g_uin = 0.0f;
    pwm_set(0.10f);
    g_uout = plant_step(0.0f);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  reset_control();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);

  uart_println("");
  uart_println("BASIC start: PI + Plant + PWM + UART");
  uart_println("Commands: help, status, mode idle|mod|cfg, kp x, ki x, ref x");
  uart_println("Boot");
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* UART RX polling: CR ends a line, LF ignored */
    uint8_t ch;
    if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
    {
#if UART_ECHO
      uart_putc(ch);
#endif
      if (ch == '\n') {
        /* ignore */
      }
      else if (ch == '\r')
      {
        uart_print("\r\n");
        g_line[g_line_len] = '\0';
        process_line(g_line);
        g_line_len = 0;
      }
      else if (g_line_len < sizeof(g_line) - 1)
      {
        g_line[g_line_len++] = (char)ch;
      }
    }

    /* Button cycles modes unless locked */
    if (g_btn_event)
    {
      g_btn_event = 0;

      if (!g_uart_config_lock)
      {
        if (g_mode == MODE_IDLE) {
          reset_control(); g_mode = MODE_MOD; uart_println("Mode -> MOD");
        } else if (g_mode == MODE_MOD) {
          reset_control(); g_mode = MODE_CONFIG; g_uart_config_lock = 1; uart_println("Mode -> CONFIG");
        } else {
          reset_control(); g_mode = MODE_IDLE; g_uart_config_lock = 0; uart_println("Mode -> IDLE");
        }
      }
      else
      {
        uart_println("Button ignored (UART lock)");
      }
    }
  }
}

/* ===================== CubeMX generated init functions below ===================== */
/* Keep YOUR CubeMX-generated versions if they already exist in your project. */
/* If you paste this file into a CubeMX project, do not duplicate these in other files. */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PC13 user button EXTI (falling) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
