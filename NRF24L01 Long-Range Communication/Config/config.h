#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* 每块从机烧录前修改，范围1~254；主机节点表必须包含相同ID。 */
#define APP_NODE_ID                 1u            /* 节点ID */
#define APP_DEFAULT_SAMPLE_MS       5000u        /* 默认采样间隔，单位ms */
#define APP_MIN_SAMPLE_MS           1000u        /* 最小采样间隔，单位ms */
#define APP_MAX_SAMPLE_MS           60000u       /* 最大采样间隔，单位ms */
#define APP_MASTER_LOST_MS          30000u       /* 主机丢失超时时间，单位ms */
#define APP_DEBUG_ENABLED           1u           /* 调试功能使能 */

#define AHT20_I2C_ADDRESS           0x38u       /* AHT20 I2C地址 */
#define BMP280_I2C_ADDRESS_PRIMARY  0x76u       /* BMP280主I2C地址 */
#define BMP280_I2C_ADDRESS_ALT      0x77u      /* BMP280备I2C地址 */

#endif
