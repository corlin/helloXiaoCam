# helloXiaoCam

这是一个基于 **ESP-IDF** 开发的 **Seeed Studio XIAO ESP32S3 Sense** 示例项目。实现了 Wi-Fi 实时视频流传输（M-JPEG）以及通过网页动态切换摄像头分辨率的功能。

## 🚀 项目特性

- **硬件完美适配**：针对 XIAO ESP32S3 Sense 的 8MB Flash 和 8MB Octal PSRAM 进行了深度优化。
- **动态分辨率切换**：支持从 **QVGA (320x240)** 到 **QXGA (2048x1536, 300万像素)** 的实时无缝切换。
- **现代化 Web UI**：内置深色模式控制面板，支持实时预览和一键调优。
- **自动传感器检测**：自动识别连接的传感器类型（支持 OV2640 和 OV3660）。

## 🛠️ 硬件需求

- **XIAO ESP32S3 Sense** (集成摄像头与扩展板)
- 建议使用支持 2.4GHz 的 Wi-Fi 网络

## 📦 快速开始

### 1. 环境准备
确保你已安装了 **ESP-IDF v5.5** 或更高版本。

```bash
# 加载 ESP-IDF 环境
. $HOME/esp/v5.5.1/esp-idf/export.sh
```

### 2. 配置 Wi-Fi
在 `main/main.c` 中修改你的 Wi-Fi 凭据：

```c
#define WIFI_SSID      "你的网络名称"
#define WIFI_PASS      "你的网络密码"
```

### 3. 编译与烧录
```bash
# 设置目标为 esp32s3
idf.py set-target esp32s3

# 编译、烧录并打开串口检查
idf.py flash monitor
```

## 🌐 访问
1. 启动后，在串口日志中找到 `Got IP: 192.168.x.x`。
2. 在浏览器访问 `http://192.168.x.x/` 即可进入控制台。

## ⚠️ 注意事项
- **带宽限制**：在 QXGA (300万像素) 分辨率下，数据量极大。如果画面卡顿或报错 `104`，请尝试在代码中调高 `jpeg_quality` 或在网页中切换回 VGA 分辨率。
- **天线**：建议接上外置天线以获得更稳定的图传性能。

## 📜 许可证
MIT
