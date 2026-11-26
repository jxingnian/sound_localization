# ESP32-C3 四麦克风声源定位与网页试听

本工程依据 `doc/SCH_Schematic1_1_2026-11-25.pdf` 的电气连接，实现 ES7210 四通道采集、可扩展音频分发、二维声源定位，以及设备热点内的四通道网页试听。

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

驱动使用 16 kHz、16 bit、四通道 TDM，数据顺序为 MIC1、MIC2、MIC3、MIC4。ESP-IDF 原生 I2C 探测使用 7 位地址 `0x41`；官方 `esp_codec_dev` 的地址参数沿用 8 位写地址约定，因此驱动转换为 `0x82` 后传入该组件。

## 组件结构

```text
components/
|-- es7210_audio/          驱动层：I2C 配置、I2S TDM、四通道 PCM 读取
|-- audio_pipeline/        中间层：引用计数帧池、独立订阅队列、多应用分发
|-- sound_localization/    应用层：TDOA、亚采样互相关、二维最小二乘、方位角
`-- audio_web_test/        测试应用：SoftAP、网页、单通道 PCM WebSocket 推流
```

`audio_pipeline` 是驱动和应用之间唯一的数据边界。声源定位和网页测试各自拥有独立订阅；后续唤醒词、录音或声音事件检测组件继续调用 `audio_pipeline_subscribe()` 即可，不需要并发读取同一个 I2S 外设。

## 网页试听（当前屏蔽）

当前硬件的 Wi-Fi 不可用，主程序已经解除 `audio_web_test` 依赖，不会初始化 Wi-Fi、SoftAP 或 HTTP 服务。固件只启动 ES7210、音频中间层和声源定位，通过串口日志周期输出定位角度、置信度和 RMS。网页测试组件源码保留在组件目录，硬件 Wi-Fi 恢复后可重新接入。

定位声音门限依据实机日志设置为 RMS `40`：实测静音底噪为 `10.8` 至 `16.8`，已捕获声音帧为 `43.7` 至 `160.6`。首个有效结果立即输出，连续有效角度最多每 100 ms 输出一次；无效状态每 10 帧输出一次。

定位热路径使用最新 160 点（10 ms）分析窗口，通过整数统计量计算归一化互相关，不在每个采样点执行软件双精度运算。定位任务会丢弃已经过时的排队帧，并在每帧后阻塞一个系统 tick，保证 ESP32-C3 单核 IDLE 任务能够运行和复位任务看门狗。

重新接入网页组件后的默认热点参数如下：

| 项目 | 值 |
| --- | --- |
| 热点名称 | `SoundLocalization` |
| 热点密码 | `sound1234` |
| 网页地址 | `http://192.168.4.1` |

页面包含 MIC 1 至 MIC 4 四个独立通道按钮，以及“启动播放”和“停止”按钮。先选择通道，再点击“启动播放”，页面会立即播放该麦克风；播放中点击其他 MIC 按钮会实时切换通道。

浏览器与设备通过 `/audio` WebSocket 通信。控制帧为 JSON 文本，音频帧为 16 kHz、16 bit、有符号小端序、单声道裸 PCM；每个二进制帧包含 480 个采样，即 30 ms 音频。页面内置 AudioWorklet 环形缓冲和线性重采样，不依赖外部网站或 CDN。

## 坐标与角度

原理图只包含电气连接，不包含 PCB 麦克风声学中心的机械坐标。`main/app_main.c` 当前明确配置为 50 mm x 50 mm 方形阵列：MIC1 左后、MIC2 右后、MIC3 左前、MIC4 右前。PCB 实际坐标确定后必须更新 `localization_config.microphones`。

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

构建产物位于 `build/output`。构建脚本只执行编译，不烧录，也不占用串口。
