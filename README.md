# STM32遥控车

项目使用了HAL库，所以可以很方便的移植。

## 硬件环境
- 开发板芯片：STM32F103VCT6
- 遥控芯片： 蓝牙HC-05
- 电机： 4个直流电机
- 电机驱动： 每个轮子输入1路PWM信号，1路方向信号

## 软件环境
- 配置生成：STM32CubeMX
- 代码编写：Jetbrains CLion
- 调试：ST-Link V2, Keil μVision5

## 项目结构
- Core： STM32CubeMX生成的代码，也是需要完成的代码。
- MDK-ARM： MDK-ARM工程文件

## 针脚分配
- 电机驱动（TIM3）： PA6, PA7, PB0, PB1
- 电机方向控制：PE7, PE9, PE11, PE13
- 蓝牙模块：UART4, Baud rate 9600: PC10, PC11
启用了DMA

## 指令格式
使用HEX发送，长度为17个字节，第0字节为指令前缀，第1-4字节每个字节代表一个轮子的方向，第5-16字节表示速度。

命令前缀：`M`：移动(Move)，`G`：读取(Get)，

正反转：0：正转；1：反转

PWM范围：0-255

### 例：
发送`M 0 0 1 0 000 255 000 255`(无空格)，
意为`移动命令 1号轮子正转, 2号轮子正转, 3号轮子反转, 4号轮子正转, 速度0，速度255，速度0，速度255`，
