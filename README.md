# ESP32-C3 四麦克风声源定位

本工程依据 `doc/SCH_Schematic1_1_2026-11-25.pdf` 的电气连接实现 ES7210 四通道采集、可扩展音频分发和二维声源定位。

## 已确认的硬件连接

| 功能 | ES7210 引脚 | ESP32-C3 | 依据 |
| --- | --- | --- | --- |
| I2C 数据 | CDATA | GPIO4 | 原理图网络标签 |
| I2C 时钟 | CCLK | GPIO5 | 原理图网络标签 |
| 主时钟 | MCLK | GPIO6 | 原理图网络标签 |
| TDM 位时钟 | SCLK | GPIO7 | 原理图网络标签 |
| TDM 帧时钟 | LRCK | GPIO10 | 原理图网络标签 |
| 四通道数据 | SDOUT1/TDMOUT | GPIO3 | 原理图网络标签 |
| I2C 地址 | AD0=1、AD1=0 | `0x41` | 原理图标注的 7 位地址 |

ES7210 的 MIC1 到 MIC4 均已接入模拟麦克风，驱动使用 16 kHz、16 bit、四通道 Philips TDM，数据顺序固定为 MIC1、MIC2、MIC3、MIC4。

## 组件结构

```text
components/
|-- es7210_audio/          驱动层：I2C 配置、I2S TDM、四通道 PCM 读取
|-- audio_pipeline/        中间层：引用计数帧池、独立订阅队列、多应用分发
`-- sound_localization/    应用层：TDOA、亚采样互相关、二维最小二乘、方位角
```

`audio_pipeline` 是驱动和应用之间唯一的数据边界。后续唤醒词、录音或声学事件检测组件通过 `audio_pipeline_subscribe()` 建立自己的队列，不需要改动 ES7210 驱动或声源定位组件。

## 坐标与角度

原理图包含电气连接，不包含 PCB 麦克风声学中心的机械坐标。`main/app_main.c` 当前明确配置为 50 mm x 50 mm 方形阵列，通道位置为：MIC1 左后、MIC2 右后、MIC3 左前、MIC4 右前。PCB 实际坐标确定后必须更新 `localization_config.microphones`。

角度以阵列中心为原点：+X 为 0 度，+Y 为 90 度，-X 为 180 度，-Y 为 270 度。

## 编译

Windows：

```powershell
.\build\sound_localization\build.ps1
```

Linux：

```bash
./build/sound_localization/build.sh
```

构建产物位于 `build/output`。构建脚本只编译，不烧录、不打开串口。
