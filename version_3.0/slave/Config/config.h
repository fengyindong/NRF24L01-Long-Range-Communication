#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ================= 节点身份与参数配置 ================= */

/* 
 * 节点ID：
 * 每块从机烧录前必须修改，范围 1~254；
 * 主机（网关）的节点表必须包含与该从机相同的ID，否则无法通信。
 */
#define APP_NODE_ID                 1u

/* 
 * 默认采样周期（毫秒）：
 * 从机上电后默认的传感器数据采集间隔。
 */
#define APP_DEFAULT_SAMPLE_MS       5000u

/* 
 * 最小采样周期（毫秒）：
 * 主机通过命令（APP_CMD_SET_SAMPLE_PERIOD）设置周期时的下限，防止过于频繁采集。
 */
#define APP_MIN_SAMPLE_MS           1000u

/* 
 * 最大采样周期（毫秒）：
 * 主机设置周期时的上限，防止过度延长导致数据时效性差。
 */
#define APP_MAX_SAMPLE_MS           60000u

/* 
 * 主机丢失判定时间（毫秒）：
 * 如果从机超过该时间未收到主机任何无线帧，则判定主机丢失，
 * 并在上报遥测数据时置位 SENSOR_MASTER_LOST 标志位。
 */
#define APP_MASTER_LOST_MS          30000u

/* 
 * 调试输出开关：
 * 1 = 使能调试串口（USART1）日志输出；
 * 0 = 关闭调试输出，节省CPU资源并减少串口干扰。
 */
#define APP_DEBUG_ENABLED           1u

/* ================= 传感器 I2C 地址配置 ================= */

/* 
 * AHT20 温湿度传感器的 I2C 从机地址（7位地址格式）。
 */
#define AHT20_I2C_ADDRESS           0x38u

/* 
 * BMP280 气压传感器的 I2C 从机地址（7位地址格式）。
 * 默认主地址：当 SDO 引脚接地时使用此地址。
 */
#define BMP280_I2C_ADDRESS_PRIMARY  0x76u

/* 
 * BMP280 气压传感器的备用 I2C 从机地址（7位地址格式）。
 * 备用地址：当 SDO 引脚接高电平（VCC）时使用此地址。
 */
#define BMP280_I2C_ADDRESS_ALT      0x77u

#endif /* APP_CONFIG_H */
