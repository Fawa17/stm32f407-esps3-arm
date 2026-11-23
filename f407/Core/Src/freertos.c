/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RobotArm.h"
#include "MPU6050.h"
#include "NetLink.h"
#include "OLED.h"
#include "Watchdog.h"
#include "Backup.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM_TASK_PERIOD_MS ROBOTARM_TICK_MS /* 10ms：PWM 插值硬节拍（100Hz） */
#define GESTURE_TASK_PERIOD_MS 20U          /* 50Hz：体感采样 */
#define NET_TASK_PERIOD_MS 5U               /* 串口命令解析：5ms 一拍，命令下发更跟手 */
#define OLED_TASK_PERIOD_MS 100U            /* 10Hz：状态画面刷新 */

#define ARM_TASK_STACK 192U     /* 单位：字（4 字节） */
#define GESTURE_TASK_STACK 384U /* 浮点解算 + HAL I2C，留足余量 */
#define NET_TASK_STACK 384U
#define OLED_TASK_STACK 320U /* vsnprintf + 1KB 显存操作 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* ---- 四个应用任务 ----
 * armTask     ：最高优先级，只做 RobotArm_Update()，20ms 一拍输出 PWM
 *               （I2C 读传感器、串口解析都在别的任务里，不能拖累它）
 * gestureTask：50Hz 读 MPU6050 → 映射关节 → 写目标角度（远程模式下自动跳过）
 * netTask    ：解析 USART1(DMA) 收到的 ESP32-S3 命令 → 改目标角度
 * oledTask   ：10Hz 刷 OLED（最低优先级，空闲时才跑）
 */
static osThreadId_t armTaskHandle;
static osThreadId_t gestureTaskHandle;
static osThreadId_t netTaskHandle;
static osThreadId_t oledTaskHandle;
static osThreadId_t wdTaskHandle;

static const osThreadAttr_t armTask_attributes = {
    .name = "armTask",
    .stack_size = ARM_TASK_STACK * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t gestureTask_attributes = {
    .name = "gestureTask",
    .stack_size = GESTURE_TASK_STACK * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static const osThreadAttr_t netTask_attributes = {
    .name = "netTask",
    .stack_size = NET_TASK_STACK * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static const osThreadAttr_t oledTask_attributes = {
    .name = "oledTask",
    .stack_size = OLED_TASK_STACK * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* 看门狗任务：优先级最低（只喂狗，绝不抢运动任务的 CPU） */
static const osThreadAttr_t wdTask_attributes = {
    .name = "wdTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* USER CODE END Variables */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void ArmTask(void *argument);
static void GestureTask(void *argument);
static void NetTask(void *argument);
static void OledTask(void *argument);
static void WdTask(void *argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* 目标角度由 gestureTask / netTask 写，armTask 读，用互斥量保护 */
    RobotArm_CreateLock();
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    armTaskHandle = osThreadNew(ArmTask, NULL, &armTask_attributes);
    gestureTaskHandle = osThreadNew(GestureTask, NULL, &gestureTask_attributes);
    netTaskHandle = osThreadNew(NetTask, NULL, &netTask_attributes);
    oledTaskHandle = osThreadNew(OledTask, NULL, &oledTask_attributes);
    wdTaskHandle = osThreadNew(WdTask, NULL, &wdTask_attributes);
    (void)armTaskHandle;
    (void)gestureTaskHandle;
    (void)netTaskHandle;
    (void)oledTaskHandle;
    (void)wdTaskHandle;
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ---- 出错保护 -------------------------------------------------------------
 * FreeRTOS 钩子：栈溢出 / 堆不够时不让系统带着坏状态继续跑舵机。
 * 停在这里（中断关闭）方便用调试器看是谁溢出了。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void) {
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

/**
 * @brief 计算本周期的唤醒 tick；如果已经落后（上一拍超时），就重新对齐，
 *        不做“追赶式”连续执行，保证节奏稳定不抖
 */
static uint32_t NextWake(uint32_t wake, uint32_t period_ms) {
    uint32_t now = osKernelGetTickCount();

    wake += period_ms;
    if ((int32_t)(wake - now) <= 0) {
        wake = now + period_ms;
    }
    return wake;
}

/**
 * @brief 舵机 PWM 插值任务（20ms 硬节拍）
 * @note  优先级最高，且只做寄存器级输出，绝不做 I2C/串口/打印，
 *        这样 6 路舵机每次走多少、什么时候走，完全确定。
 */
static void ArmTask(void *argument) {
    uint32_t wake = osKernelGetTickCount();

    (void)argument;

    for (;;) {
        RobotArm_Update(); /* 梯形加减速逼近目标 + 写 CCR */
        Watchdog_KeepAlive(WD_TASK_ARM);
        wake = NextWake(wake, ARM_TASK_PERIOD_MS);
        (void)osDelayUntil(wake);
    }
}

/**
 * @brief 体感任务（50Hz）：读 MPU6050 → 映射 6 个关节 → 写目标角度
 * @note  I2C 一次突发读 14 字节，100kHz 下约 1.6ms，允许被 ArmTask 抢占
 */
static void GestureTask(void *argument) {
    uint32_t wake = osKernelGetTickCount();

    (void)argument;

    for (;;) {
        MPU6050_GestureStep();
        Watchdog_KeepAlive(WD_TASK_GESTURE);
        wake = NextWake(wake, GESTURE_TASK_PERIOD_MS);
        (void)osDelayUntil(wake);
    }
}

/**
 * @brief 网络任务：解析 USART1(DMA) 收到的 ESP32-S3 命令
 * @note  NetLink_Poll 内部只在 DMA 环形缓冲上推进读指针，不阻塞
 */
static void NetTask(void *argument) {
    (void)argument;

    for (;;) {
        NetLink_Poll();
        Watchdog_KeepAlive(WD_TASK_NET);
        (void)osDelay(NET_TASK_PERIOD_MS);
    }
}

/**
 * @brief OLED 状态监视任务（10Hz）
 * @note  优先级最低，只在 CPU 空闲时刷屏；屏没接时 OLED_StatusTask 直接返回
 */
static void OledTask(void *argument) {
    uint32_t wake = osKernelGetTickCount();

    (void)argument;

    for (;;) {
        OLED_StatusTask();
        Watchdog_KeepAlive(WD_TASK_OLED);
        wake = NextWake(wake, OLED_TASK_PERIOD_MS);
        (void)osDelayUntil(wake);
    }
}

/**
 * @brief 看门狗任务：四个任务都按时报到才喂狗
 * @note  给 2s 启动宽限（上电时主线程还在做舵机软启动和回中位），
 *        之后每 200ms 检查一次；任何一个任务超期就不再喂狗，让 IWDG 复位。
 */
static void WdTask(void *argument) {
    uint32_t wake = osKernelGetTickCount();
    uint8_t healthy_done = 0U;

    (void)argument;

    (void)osDelay(pdMS_TO_TICKS(2000));
    Watchdog_Start();

    for (;;) {
        if (Watchdog_AllAlive()) {
            Watchdog_Feed();
        }

        /* 启动健康上报：跑满 3 秒说明各任务都起来了，先盖 "SEAL" 印章
         * （引导程序据此把启动计数归零），再等机械臂停稳把当前镜像备份到 W25Q16。
         * 分数次喂给低优先级任务做，不阻塞舵机节拍。 */
        if (!healthy_done && osKernelGetTickCount() > 3000U) {
            Watchdog_Feed();
            if (Backup_Healthy() != 0U) {
                healthy_done = 1U;
            }
        }

        wake = NextWake(wake, 200U);
        (void)osDelayUntil(wake);
    }
}

/* USER CODE END Application */
