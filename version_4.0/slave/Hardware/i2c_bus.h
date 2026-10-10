#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>

/* ============================================================
 * 模拟 I2C 总线驱动接口
 *
 * 使用 GPIO 模拟 I2C 时序，引脚为：
 *   - PB6：SCL（时钟线）
 *   - PB7：SDA（数据线）
 *
 * 硬件要求：
 *   - 两根线必须外接上拉电阻（典型 4.7kΩ～10kΩ）。
 *   - 引脚配置为开漏输出，依靠外部上拉形成高电平。
 *
 * 地址约定：
 *   - 所有接口的 address7 均为 7 位从机地址（不含读写位）。
 *   - 返回 1 表示操作成功；返回 0 表示 NACK 或参数错误。
 * ============================================================ */

/* ============================================================
 * 总线初始化与恢复
 * ============================================================ */

/**
 * @brief 初始化 PB6/PB7 为开漏输出，释放总线并尝试执行一次总线恢复。
 *
 * 初始化后 SCL 和 SDA 均为高电平（释放状态）。如果总线上有从设备
 * 在传输过程中被复位卡住，会自动调用 i2c_bus_recover() 尝试恢复。
 */
void i2c_bus_init(void);

/**
 * @brief 总线恢复：输出最多 9 个 SCL 脉冲和 STOP，尝试释放被卡住的 SDA。
 *
 * 应用场景：
 *   主控在 I2C 传输过程中被复位，从设备可能一直拉低 SDA 等待时钟，
 *   导致后续通信无法开始。此函数通过发送额外时钟脉冲让从设备把剩余
 *   数据位移出，然后发 STOP 释放总线。
 *
 * @return 1 表示恢复后 SDA 为高（总线已释放）；0 表示总线仍被拉低。
 */
uint8_t i2c_bus_recover(void);

/* ============================================================
 * 写操作
 * ============================================================ */

/**
 * @brief 向 7 位地址从机连续写入 length 字节。
 *
 * 时序：START → 地址+W → 数据1 → 数据2 → ... → STOP
 *
 * @param address7 7 位从机地址（不含读写位）
 * @param data     待写入的数据缓冲区
 * @param length   要写入的字节数
 * @return 1 表示全部字节都收到 ACK；0 表示任意阶段收到 NACK
 */
uint8_t i2c_write(uint8_t address7, const uint8_t *data, uint8_t length);

/**
 * @brief 向指定 8 位寄存器写入一个字节。
 *
 * 是 i2c_write 的便捷封装，适用于带寄存器地址的传感器（如 BMP280）。
 * 时序：START → 地址+W → 寄存器地址 → 数据 → STOP
 *
 * @param address7 7 位从机地址
 * @param reg      寄存器地址
 * @param value    要写入的值
 * @return 1 成功；0 失败
 */
uint8_t i2c_write_reg(uint8_t address7, uint8_t reg, uint8_t value);

/* ============================================================
 * 读操作
 * ============================================================ */

/**
 * @brief 从命令式 I2C 设备直接读取连续数据（不带寄存器地址）。
 *
 * 主要用于 AHT20 等命令式接口：先发送测量命令，再直接读回结果。
 * 时序：START → 地址+R → 数据1（主机 ACK）→ ... → 数据n（主机 NACK）→ STOP
 *
 * @param address7 7 位从机地址
 * @param data     输出缓冲区
 * @param length   要读取的字节数
 * @return 1 成功；0 失败（参数错误或地址阶段 NACK）
 */
uint8_t i2c_read(uint8_t address7, uint8_t *data, uint8_t length);

/**
 * @brief 从指定 8 位寄存器开始连续读取数据。
 *
 * 主要用于 BMP280 等需要先指定寄存器再读的传感器。
 * 时序：START → 地址+W → 寄存器地址 → 重复 START → 地址+R → 数据1...数据n → STOP
 *
 * @param address7 7 位从机地址
 * @param reg      寄存器地址
 * @param data     输出缓冲区
 * @param length   要读取的字节数
 * @return 1 成功；0 表示任意阶段未应答
 */
uint8_t i2c_read_regs(uint8_t address7, uint8_t reg, uint8_t *data, uint8_t length);

#endif /* I2C_BUS_H */