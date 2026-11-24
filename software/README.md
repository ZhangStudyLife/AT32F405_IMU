# AT32F405 IMU 固件

目标芯片为 AT32F405KBU7-4。构建使用 Windows 版 Keil MDK 5，已验证的工具版本为 µVision 5.42 和 Arm Compiler 6.23。

1. 首次使用时，双击仓库随附的 [ArteryTek 器件包 2.1.6](vendor/ArteryTek.AT32F402_405_DFP.2.1.6.pack)，按 Keil 提示安装。器件包包含本项目的系统文件、外设驱动、调试寄存器描述和 128 KiB Flash 下载算法。
2. 打开 [imu_readout.uvprojx](imu_readout.uvprojx)，选择 `AT32F405_IMU` 目标，执行 **Rebuild**。µVision 按选定芯片定位器件包，CMSIS 5.9.0 的 Cortex-M4 头文件随工程提供，支持离线构建。
3. 编译产物在 `objects/` 下，烧录文件为 `imu_readout.hex`，调试文件为 `imu_readout.axf`。模块接线和串口命令见 [UART 协议](UART协议.md)。

随库器件包取自 [ArteryTek 官方仓库](https://github.com/ArteryTek/Open-CMSIS-Pack/blob/main/ArteryTek.AT32F402_405_DFP.2.1.6.pack)。Arm 头文件取自 [CMSIS 5.9.0](https://github.com/ARM-software/CMSIS_5/tree/5.9.0/CMSIS/Core/Include)，许可见 [Apache 2.0](vendor/CMSIS/LICENSE.txt)。
