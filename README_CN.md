# 广州市星翼电子科技有限公司（ALIENTEK）

## 基础例程

- **开发平台**: ALIENTEK T5 MINI Board
- **开发目的**: 如何驱动OV2640把图像显示在LCD上

---

## 硬件资源与引脚分配
- RGBLCD        SPILCD
​	 - R3: P23
​	 - R4: P22
​	 - R5: P21
​	 - R6: P20
​	 - R7: P19
​	 - G2: P42    - DC/WR
​	 - G3: P41
​	 - G4: P40
​	 - G5: P26
​	 - G6: P25
​	 - G7: P24
​	 - B3: P47
​	 - B4: P46    - SDA
​	 - B5: P45    - CS
​	 - B6: P44    - SCL
​	 - B7: P43    - RST
​	 - PCLK: P14
​	 - DE: P16
​	 - BL: P9     - PWR
​	 - RST: P27

- CAMERA
  - I2C_SCL: P13
  - I2C_SDA: P15
  - DVP_RST: P8
  - DVP_PWDN: P7
  - DVP_D0: P32
  - DVP_D1: P33
  - DVP_D2: P34
  - DVP_D3: P35
  - DVP_D4: P36
  - DVP_D5: P37
  - DVP_D6: P38
  - DVP_D7: P39
  - DVP_PCLK: P29
  - DVP_VSYNC: P31
  - DVP_HSYNC: P30

---

## 运行结果
- 可在屏幕中显示摄像头实时的图像数据

---

## 注意
- 无

---

## 云台目标追踪扩展（本仓库新增）

在原始摄像头显示例程基础上，增加了**视觉目标提取 + 串口舵机云台追踪**闭环，以及运行时调参命令行与上位机串口工具。

### 功能概述
- 视觉管线从 YUYV 帧中提取目标中心（矩形四角 / 色度窗掩膜 / 同心圆环质心，可选模式）
- 增量式 P 控制驱动两路舵机（水平/俯仰），使目标中心收敛到画面中心
- 无完整四边形时进入**捕获模式**：取不贴边的最大目标色连通域质心，小步"走向"目标
- 全部视觉/追踪参数运行时可调（日志口 shell 命令），并带画质 watchdog 自动恢复相机

### 硬件连接与供电
- **舵机串口**: UART0（TX=GPIO11 → 舵机模块 RX，RX=GPIO10），波特率 **9600** 8N1（`SERVO_UART_BAUDRATE`，按舵机模块实际修改）
- **舵机编号**: 2 = 俯仰(tilt)，3 = 水平(pan)；初始角度 tilt=170、pan=90（`servo.h`）
- **日志/命令串口**: UART1（默认日志口，460800）。BK shell 位于 CP 核，**AP 应用命令需加 `ap_cmd` 前缀**
- **供电警告**: 舵机/舵机模块必须**独立 5V≥2A 供电并与开发板共地**。从板载 5V 取电会在舵机启动瞬间拉垮电源导致整板 brownout 重启（实测反复复现）

舵机协议帧（6 字节）：

| 字节 | 内容 | 说明 |
| :--- | :--- | :--- |
| 1 | 0x24 | 包头 `$` |
| 2 | servonum + 64 | 舵机编号（2→66，3→67） |
| 3 | '0'+angle/100 | 角度百位 ASCII |
| 4 | '0'+angle/10%10 | 角度十位 ASCII |
| 5 | '0'+angle%10 | 角度个位 ASCII |
| 6 | 0x23 | 包尾 `#` |

### 视觉管线（`components/BSP/CAMERA/vision.c`）
- `cmode 0`: 灰度阈值（Otsu 或手动 `thr`）→ Canny → 轮廓追踪 → 四边形打分（角度/面积/边距）
- `cmode 1`（**默认**）: 色度窗分类 `u_lo≤U≤u_hi && v_lo≤V≤v_hi && y_lo≤Y≤y_hi`（默认 U/V∈[180,255]，适用于白平衡偏色场景下纸面与背景色度分离）→ 形态学 → 四边形检测；无四边形时启用捕获模式
- `cmode 2`: 同心圆环质心：暗亮度 + `|U−V|>20` 色度门 → 连通域（bbox 15~90px）质心
- `morph`: 3x3 开+闭运算去椒盐噪声；`eps`: 多边形近似容差(百分比x100)；`margin`: 四角离边最小距离；`cphase`: 色度有效行相位(mod 4，传感器隔行有效时消除条纹)
- **画质 watchdog**: 连续 3 次统计灰度动态范围 <40 判定传感器配置失效，自动 `camera_restart`（断电复位+重开+ID 校验重试，30s 防抖）

### 运行时调参命令（日志口 shell，AP 命令加 `ap_cmd` 前缀）
| 命令 | 功能 |
| :--- | :--- |
| `vget` | 打印全部视觉/舵机参数与最近识别结果 |
| `vset <name> <value>` | 设置参数（见下表） |
| `vservo <num> <angle>` | 手动发舵机帧（同步内部角度） |
| `vimg` | 按需 dump 二值掩膜（GRAY8 hex） |
| `vraw` | 按需 dump 原始 YUYV（含色度） |
| `vcam` | 相机断电复位+重新初始化（重试至 ID 正确） |

视觉参数：`thr`(-1=Otsu, 0~255 手动)、`canny_lo`、`canny_hi`、`min_area`、`margin`、`morph`、`eps`、`proc`(0= bypass 全部算法, LCD 显示原始灰度)、`cmode`、`u_lo/u_hi/v_lo/v_hi/y_lo/y_hi`、`cphase`
舵机参数：`track`(0/1)、`interval`(ms)、`deadzone`(px)、`kp`(x100, 度/像素)、`step`(单步最大度数)、`pandir`、`tiltdir`(±1)

### 上位机工具（`tools/`）
```bash
# 实时日志 + 指令注入服务（TCP 127.0.0.1:7777）
python3 tools/serial_console.py monitor --port /dev/ttyACM0 --baud 460800 --out monitor.log
# 发指令并捕获回复（经 monitor daemon；无 daemon 时直连串口）
python3 tools/serial_console.py send ap_cmd vset kp 2 --ctl 7777
# 交互式终端
python3 tools/serial_console.py shell --port /dev/ttyACM0 --baud 460800
# 解码 dump 图像（GRAY8/YUYV → PNG，YUYV 附带色度统计与灰度图）
python3 tools/decode_serial_img.py monitor.log out.png
```

### 构建与烧录
```bash
. ./export.sh          # 仓库根目录初始化环境
tos.py build           # 本工程目录下
tos.py flash -p /dev/ttyACM0
tos.py monitor -p /dev/ttyACM0 -b 460800
```

### 追踪调参经验与已知限制
- 默认柔和增益：`kp=2`(0.02°/px)、`step=1`、`interval=300`、`deadzone=12`。视觉帧率仅 ~3fps 且舵机有滞后，增益过大会产生极限环振荡；先降 `kp/step`、加大 `interval` 再谈精度
- 方向符号用单轴实验测定：手动 `vservo` 单轴步进观察目标移动方向，再定 `pandir/tiltdir`
- 视场较窄（~25°）：目标距离建议 1m 左右，使矩形占画面约一半且四边留余量
- 舵机模块存在丢步/回差：开环姿态搜索不可靠，依赖闭环捕获模式逐步收敛
- 俯仰行程有限：若 tilt 饱和（180）目标仍偏下，需物理降低目标或垫高云台
- 传感器初始化偶发失败（分辨率错误中断/灰度洗白）：boot 与 `vcam` 带重试，画质 watchdog 兜底

---

## Company Information
- 公司名称：广州市星翼电子科技有限公司（正点原子）
- 电话号码：020-38271790
- 传真号码：020-36773971
- 公司网址：www.alientek.com
- 购买链接：[zhengdianyuanzi.tmall.com](https://zhengdianyuanzi.tmall.com)
- 技术论坛：[http://www.openedv.com/forum.php](http://www.openedv.com/forum.php)
- 最新文档：[www.openedv.com/docs/index.html](http://www.openedv.com/docs/index.html)
- 在线视频：[www.yuanzige.com](http://www.yuanzige.com)
- B 站视频：[space.bilibili.com/394620890](https://space.bilibili.com/394620890)
- 微信公众号：[mp.weixin.qq.com/s/y--mG3qQT8gop0VRuER9bw](https://mp.weixin.qq.com/s/y--mG3qQT8gop0VRuER9bw)
- 抖    音: [douyin.com/user/MS4wLjABAAAAi5E95JUBpqsW5kgMEaagtIITIl15hAJvMO8vQMV1tT6PEsw-V5HbkNLlLMkFf1Bd](https://www.douyin.com/user/MS4wLjABAAAAi5E95JUBpqsW5kgMEaagtIITIl15hAJvMO8vQMV1tT6PEsw-V5HbkNLlLMkFf1Bd)