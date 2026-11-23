/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 六轴机械臂主程序（STM32F407VET6 + FreeRTOS/CMSIS-RTOS2）
 ******************************************************************************
 * 结构：初始化在本文件里一次做完，之后交给 freertos.c 里的四个任务：
 *
 *   每 5ms   netTask      NetLink_Poll()        解析 ESP32-S3 发来的串口命令
 *   每 20ms  gestureTask  MPU6050_GestureStep() 读姿态并换算成目标角度（远程模式自动跳过）
 *   每 20ms  armTask      RobotArm_Update()     梯形速度插值 → 写 CCR → PWM
 *   每 100ms oledTask     OLED_StatusTask()     刷状态屏（链路/模式/角度/姿态）
 *
 * 上电顺序：串口 → OLED → 六路 PWM 分时软启动 + 回中位(1.5s) → MPU6050 校准
 *           （传感器没接时会每秒重试，期间机械臂照常接受网络命令）
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RobotArm.h"
#include "MPU6050.h"
#include "NetLink.h"
#include "OLED.h"
#include "Param.h"
#include "Backup.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    /* IAP：应用被引导程序拷到 0x08010000，中断向量表必须跟着搬。
       这一行要放在 HAL_Init() 之前（HAL_Init 里会开中断），否则第一次中断会跳到引导程序的向量。 */
    SCB->VTOR = 0x08010000U;

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
    MX_TIM3_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */

    /* 0. 串口链路（ESP32-S3）：USART1 异步 115200 + DMA2_Stream2 循环接收。
     *    先开它，这样即使后面回中位要 2 秒多，上位机发的命令也不会丢（DMA 缓冲着）。 */
    NetLink_Init();

    /* 0.5 OLED 状态屏（I2C1，PB6/PB7）：屏没接也不影响机械臂，只是不显示 */
    (void)OLED_Init();

    /* 0.8 从板载 W25Q16 读回标定表/速度（没有芯片或数据无效就用默认值，
     *     不影响后续流程）。放在 RobotArm_Init 之前，这样回中位就已经按标定走。 */
    (void)Param_Init();

    /* 0.9 固件备份区（W25Q16）：读备份头，供"新固件起不来就回滚"用。
     *     没插芯片时返回 1，健康印章照样会盖（只是没法回滚），其余功能不受影响。 */
    (void)Backup_Init();

    /* 1. 六路 PWM 分时软启动（每路间隔 200ms，避免 6 个舵机同时启动的冲击电流）
     * 2. 在 1.5s 内线性插值把六轴送回中位 90°（阻塞，走完才继续） */
    RobotArm_Init();

    /* 3. MPU6050 初始化 + 陀螺零偏校准（约 0.4s，此时传感器要静止）
     * 4. 把当前姿势记为体感中位基准（约 0.2s，期间不要动）+ 夹爪张开
     * 传感器没接好时每秒重试一次，期间照常响应网络命令（不会死机跑飞） */
    while (MPU6050_GestureInit() != 0) {
        uint8_t k;

        /* 约 1s，期间保持 20ms 拍子服务网络命令 */
        for (k = 0; k < 50U; k++) {
            NetLink_Poll();
            RobotArm_Update();
            HAL_Delay(20);
        }
    }

    /* 丢掉启动期间积压的串口数据，从干净状态开始解析 */
    NetLink_Flush();
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 调度器已经接管：这里永远不会执行到。
         * 四个任务的节拍在 freertos.c 里（5ms/20ms/20ms/100ms）。 */
        osDelay(1000);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM4) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
