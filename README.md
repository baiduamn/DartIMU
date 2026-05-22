# DartIMU — 视觉制导飞镖系统主控板核心固件

本仓库为具备视觉制导能力的飞镖系统主控板核心固件片段。系统基于 **STM32G431C6** 开发，采用面向对象编程（OOP）思想设计，实现了多传感器高频数据流稳定流转、姿态估计、高速 Flash 数据记录及视觉目标协议帧解析。

---

## 核心功能模块

1. **十轴 IMU 驱动与姿态融合**
   * **ICM45686（SPI）**：高频加速度/角速度数据采集，支持启动时陀螺仪自动零偏校准。
   * **QMC5883（I2C）**：高频磁强计数据采集，内建软件硬铁磁干扰极值校准算法。
   * **BMP388（I2C）**：出厂标定参数补偿的气压/温度解算，提供厘米级相对高度估计。
   * **Mahony AHRS**：实时九轴姿态融合解算，输出高稳定性的四元数及欧拉角。

2. **W25Q128 飞行黑匣子数据存盘**
   * 基于 SPI2 实现对 W25Q128 芯片的读写控制，支持扇区擦除、页编程。
   * 采用内存双缓存机制降低实时控制回路中的 Flash 写入抖动，提供完备的数据复盘分析支撑。

3. **OpenMV 视觉数据接收解析链路**
   * 基于 USART1 接收 OpenMV 输出的视觉检测坐标与大小。
   * **接收自愈容错机制**：
     * **字节间隔超时重置**：数据链如果中断超过 10 ms 状态机自动复位，防止数据帧对齐出错。
     * **硬件错误自动恢复**：注册 `HAL_UART_ErrorCallback`，在突发噪声干扰引起 Overrun（溢出）等硬件错误时，自动清理标记并重启中断，防止通信挂死。

---

## 项目层级目录

```text
├── cmake/                      # CMake 构建配置目录
│   └── stm32cubemx/            # CubeMX 生成的模块构建列表
├── Core/
│   ├── Inc/                    # CubeMX 系统初始化头文件
│   └── Src/                    # 应用主入口与系统初始化源文件 (main.c, stm32g4xx_it.c等)
├── Drivers/
│   ├── BSP/                    # 核心板级支持包
│   │   ├── Flash/              # W25Q128 驱动 (w25q128.c/h)
│   │   ├── IMU/                # 十轴 IMU 驱动组 (BMP388, ICM45686, QMC5883及融合层)
│   │   └── OpenMV/             # 视觉串口状态机解析器 (openmv_parser.c/h)
│   └── Interface/              # 硬件延迟基础接口 (delay.c/h)
├── Jlink_RTT/                  # SEGGER RTT 调试输出输出工具包
├── CMakeLists.txt              # 构建主脚本
├── CMakePresets.json           # 构建预设配置
├── STM32G431XX_FLASH.ld        # 链接脚本
└── startup_stm32g431xx.s       # 启动文件
```

---

## 面向对象（C-OOP）架构设计

为降低底层代码和上层算法的耦合度，驱动层采用统一的方法指针结构进行高度解耦：

```c
/* 十轴 IMU 对象封装示例 */
typedef struct TenAxisIMU {
    ICM45686_Device imu;
    BMP388_Device baro;
    QMC5883_Device mag;
    
    float q[4];
    float yaw;
    float pitch;
    float roll;
    float altitude;
    
    HAL_StatusTypeDef (*Init)(struct TenAxisIMU *self);
    HAL_StatusTypeDef (*Update)(struct TenAxisIMU *self);
    void (*GetEuler)(struct TenAxisIMU *self, float *yaw, float *pitch, float *roll);
} TenAxisIMU;
```

---

| 资源 | 已使用大小 | 芯片容量限制 | 占用率 |
| :--- | :--- | :--- | :--- |
| **SRAM** | 4.28 KB (4392 Bytes) | 32 KB | **13.09%** |
| **FLASH** | 60.04 KB (61480 Bytes) | 128 KB | **45.81%** |

