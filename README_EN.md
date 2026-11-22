# Guangzhou Xingyi Electronic Technology Co., Ltd. (ALIENTEK)

## BASE DEMO

- **Demo Platform**: ALIENTEK T5 MINI Board
- **Demo Purpose**: How to drive OV2640 to display images on LCD

---

## Hardware Resources & Pin Assignment
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

## Execution Results
- Real-time camera image data can be displayed on the screen

---

## Notes
- None

---

## Gimbal Target Tracking Extension (added in this repository)

On top of the original camera-display demo, this project adds a **vision target extraction + UART servo gimbal tracking** loop, a runtime tuning CLI, and host-side serial tools.

### Overview
- Vision pipeline extracts the target center from YUYV frames (rectangle corners / chroma-window mask / concentric-ring centroid, selectable modes)
- Incremental P-control drives two servos (pan/tilt) to converge the target center to the image center
- **Acquisition mode** when no full quadrilateral is visible: centroid of the largest non-border-touching target-colored component, walking toward the target in small steps
- All vision/tracking parameters are runtime-tunable via log-uart shell commands; a picture-quality watchdog auto-recovers the camera

### Hardware Wiring & Power
- **Servo UART**: UART0 (TX=GPIO11 → servo module RX, RX=GPIO10), baud **9600** 8N1 (`SERVO_UART_BAUDRATE`, match your servo module)
- **Servo numbers**: 2 = tilt, 3 = pan; initial angles tilt=170, pan=90 (`servo.h`)
- **Log/command UART**: UART1 (default log port, 460800). The BK shell runs on the CP core; **AP application commands need the `ap_cmd` prefix**
- **Power warning**: servos / servo module MUST use an **independent 5V ≥2A supply with common ground** to the board. Powering them from the board's 5V rail causes brownout reboots at servo start-up (repeatedly observed)

Servo protocol frame (6 bytes):

| Byte | Content | Description |
| :--- | :--- | :--- |
| 1 | 0x24 | header `$` |
| 2 | servonum + 64 | servo index (2→66, 3→67) |
| 3 | '0'+angle/100 | hundreds digit ASCII |
| 4 | '0'+angle/10%10 | tens digit ASCII |
| 5 | '0'+angle%10 | units digit ASCII |
| 6 | 0x23 | tail `#` |

### Vision Pipeline (`components/BSP/CAMERA/vision.c`)
- `cmode 0`: luma threshold (Otsu or manual `thr`) → Canny → contour tracing → quadrilateral scoring (angles/area/border margin)
- `cmode 1` (**default**): chroma window `u_lo≤U≤u_hi && v_lo≤V≤v_hi && y_lo≤Y≤y_hi` (default U/V∈[180,255]; separates paper from background under shifted white balance) → morphology → quad detection; acquisition mode when no quad
- `cmode 2`: concentric-ring centroid: dark luma + `|U−V|>20` chroma gate → components (bbox 15~90px) centroid
- `morph`: 3x3 open+close despeckle; `eps`: polygon approx epsilon (percent x100); `margin`: min corner distance to border; `cphase`: chroma valid-row phase (mod 4, removes row stripes when sensor chroma is interlaced)
- **Quality watchdog**: 3 consecutive stats with luma dynamic range <40 ⇒ sensor misconfiguration ⇒ automatic `camera_restart` (power-cycle + reopen + ID-check retries, 30s lockout)

### Runtime Tuning Commands (log-uart shell; prefix AP commands with `ap_cmd`)
| Command | Function |
| :--- | :--- |
| `vget` | dump all vision/servo params and last detection result |
| `vset <name> <value>` | set a parameter (table below) |
| `vservo <num> <angle>` | manual servo frame (syncs internal angle state) |
| `vimg` | on-demand binary mask dump (GRAY8 hex) |
| `vraw` | on-demand raw YUYV dump (with chroma) |
| `vcam` | camera power-cycle + re-init (retries until sensor ID reads back) |

Vision params: `thr`(-1=Otsu, 0~255 manual), `canny_lo`, `canny_hi`, `min_area`, `margin`, `morph`, `eps`, `proc`(0=bypass all processing, LCD shows raw gray), `cmode`, `u_lo/u_hi/v_lo/v_hi/y_lo/y_hi`, `cphase`
Servo params: `track`(0/1), `interval`(ms), `deadzone`(px), `kp`(x100, deg/pixel), `step`(max deg per update), `pandir`, `tiltdir`(±1)

### Host Tools (`tools/`)
```bash
# real-time log + command injection server (TCP 127.0.0.1:7777)
python3 tools/serial_console.py monitor --port /dev/ttyACM0 --baud 460800 --out monitor.log
# send a command and capture the reply (via monitor daemon; direct serial fallback)
python3 tools/serial_console.py send ap_cmd vset kp 2 --ctl 7777
# interactive terminal
python3 tools/serial_console.py shell --port /dev/ttyACM0 --baud 460800
# decode dumped images (GRAY8/YUYV → PNG; YUYV also prints chroma stats + gray png)
python3 tools/decode_serial_img.py monitor.log out.png
```

### Build & Flash
```bash
. ./export.sh          # init environment at repo root
tos.py build           # inside this project directory
tos.py flash -p /dev/ttyACM0
tos.py monitor -p /dev/ttyACM0 -b 460800
```

### Tuning Experience & Known Limitations
- Default gentle gains: `kp=2` (0.02°/px), `step=1`, `interval=300`, `deadzone=12`. Vision runs at ~3fps with servo lag; excessive gain causes limit-cycle oscillation — reduce `kp/step` and raise `interval` first
- Determine direction signs with single-axis experiments: step one servo via `vservo`, observe target motion, then set `pandir/tiltdir`
- Narrow FOV (~25°): keep the target ~1m away so the rectangle fills about half the frame with border margin
- Servo module suffers lost steps / backlash: open-loop pose searches are unreliable; rely on closed-loop acquisition to converge
- Limited tilt travel: if tilt saturates (180) with the target still low, physically lower the target or raise the gimbal mount
- Sensor init occasionally fails (resolution-error IRQ / washed-out luma): boot and `vcam` retry; quality watchdog as fallback

---

## Company Information
- Company Name: Guangzhou Xingyi Electronic Technology Co., Ltd. (ALIENTEK)
- Tel: 020-38271790
- Fax: 020-36773971
- Website: www.alientek.com
- Purchase: [zhengdianyuanzi.tmall.com](https://zhengdianyuanzi.tmall.com)
- Technical Forum: [http://www.openedv.com/forum.php](http://www.openedv.com/forum.php)
- Latest Docs: [www.openedv.com/docs/index.html](http://www.openedv.com/docs/index.html)
- Online Video: [www.yuanzige.com](http://www.yuanzige.com)
- Bilibili: [space.bilibili.com/394620890](https://space.bilibili.com/394620890)
- WeChat Official Account: [mp.weixin.qq.com/s/y--mG3qQT8gop0VRuER9bw](https://mp.weixin.qq.com/s/y--mG3qQT8gop0VRuER9bw)
- Douyin: [douyin.com/user/MS4wLjABAAAAi5E95JUBpqsW5kgMEaagtIITIl15hAJvMO8vQMV1tT6PEsw-V5HbkNLlLMkFf1Bd](https://www.douyin.com/user/MS4wLjABAAAAi5E95JUBpqsW5kgMEaagtIITIl15hAJvMO8vQMV1tT6PEsw-V5HbkNLlLMkFf1Bd)
