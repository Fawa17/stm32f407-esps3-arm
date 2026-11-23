/** NetLink.h - 上位机(ESP32-S3) <-> STM32 串口协议 物理层：USART1，PA9=TX / PA10=RX，115200 8N1，RX
 * 用 DMA2_Stream2 通道4 循环接收。 */
#ifndef __NETLINK_H
#define __NETLINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 运行模式 */
#define NET_MODE_GESTURE 0 /* MPU6050 体感跟随 */
#define NET_MODE_REMOTE 1  /* 听上位机 P 命令 */

/* 版本号：上位机发 Q 时回给 S3，Backup.c 也拿它算印章版本 */
#define NET_FW_VERSION "f4-v7"

/* 链路参数 */
#define NET_RX_BUF_SIZE 512U      /* 115200 下约 44ms 的缓冲，够一次扇区擦除用 */
#define NET_LINE_MAX 128U         /* 一行上限：IAP 的 W 帧（48 字节 -> 96 hex + 头）约 105 字符 */
#define NET_TX_TIMEOUT_MS 20U     /* 发送超时 */
#define NET_LINK_TIMEOUT_MS 3000U /* 多久没收到上行算断链 */
#define NET_HEARTBEAT_MS 500U     /* H,1 打开后的上报周期 */

/** 初始化：接管 USART1（MX_USART1_UART_Init 之后再调）+ 启动 DMA 接收 */
void NetLink_Init(void);

/** 在主循环/任务里周期调用：取 DMA 缓冲区、拆行、解析 */
void NetLink_Poll(void);

/** 丢弃缓冲区里已有的旧数据（上电初始化完清一次） */
void NetLink_Flush(void);

/** 当前模式：NET_MODE_GESTURE / NET_MODE_REMOTE */
uint8_t NetLink_GetMode(void);

/** 切模式，同时回一帧状态 */
void NetLink_SetMode(uint8_t mode);

/** 最近 NET_LINK_TIMEOUT_MS 内有没有收到过完整命令行 */
uint8_t NetLink_LinkAlive(void);

/** 累计收到 / 出错的命令条数 */
uint32_t NetLink_GetCmdCount(void);
uint32_t NetLink_GetErrCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __NETLINK_H */
