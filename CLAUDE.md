# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此仓库中工作时提供指导。

## 项目目标

**无感 FOC 空调压缩机驱动** —— 基于 ESP32 的无位置传感器磁场定向控制（FOC），用于驱动空调压缩机中的永磁同步电机（PMSM）。

- 目标电机：PMSM 压缩机（典型参数：7 对极，额定转速 1000-7200 RPM）
- 控制方式：无感 FOC（无霍尔传感器/编码器，依赖反电动势或磁链观测器估算转子位置）
- 硬件平台：ESP32 + MCPWM 三相逆变器

### 当前状态与待实现

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| SVPWM 逆变器驱动 (`esp_svpwm`) | ✅ 完成 | MCPWM 三相中心对齐 PWM 输出 |
| FOC 坐标变换 (`esp_foc`) | ✅ 完成 | 逆 Park、逆 Clarke、SVPWM 7段式调制 |
| FocDriver 基础框架 | ⚠️ 框架已有 | 定时器+任务框架就绪，但控制回路未闭合 |
| 电流采样 | ❌ 未实现 | 需要 ADC 双电阻/三电阻采样 |
| Clarke/Park 正变换 | ❌ 未实现 | 三相电流 → αβ → dq |
| 转子位置估算 | ❌ 未实现 | 滑模观测器(SMO) 或 磁链观测器 |
| 速度环 PI | ❌ 未实现 | 速度闭环控制 |
| 电流环 PI | ❌ 未实现 | d/q 轴电流闭环控制 |
| 启动策略 | ❌ 未实现 | 开环强拖 → 闭环切换（无感启动） |

## 构建系统

- **框架**: ESP-IDF v6.0.0，基于 CMake
- **目标芯片**: ESP32 (Xtensa)
- **工具链**: `xtensa-esp-elf`（安装路径 `~/.espressif/tools/`）
- **构建**:
  ```bash
  idf.py build              # 完整构建
  idf.py app                # 仅构建 app（跳过 bootloader）
  ```
- **烧录 & 监视**:
  ```bash
  idf.py flash monitor       # 烧录到设备并打开串口监视器
  ```
- **配置**:
  ```bash
  idf.py menuconfig          # 交互式 Kconfig 配置；写入 sdkconfig
  ```
- 项目根目录的 `sdkconfig` 是已提交的 SDK 配置。`sdkconfig.ci` 为空 — CI 构建应提供自己的配置。
- 编译输出位于 `build/`。

## 高层架构

### 项目结构

```
CMakeLists.txt              # 顶层；包含 IDF project.cmake，项目名 "FocCompressor"
main/
  app_main.cpp              # 入口点（FreeRTOS app_main）
components/
  motor_foc_driver/          # FOC 电机驱动组件（所有 .cpp → 单个静态库）
  project_conf/              # 纯头文件的硬件配置组件（无 SRCS）
```

### 组件依赖链

```
main
 └── motor_foc_driver
      ├── project_conf      （引脚分配、PWM 时序、极对数）
      ├── driver            （gpio、mcpwm）
      ├── esp_timer         （高频控制循环定时器）
      └── esp_driver_gpio / esp_driver_mcpwm
```

`motor_foc_driver` 组件通过 glob 匹配目录下所有 `*.cpp` 文件。新增 `.cpp` 文件会自动被包含 — 无需修改 CMakeLists.txt。

### FOC 控制循环

电机控制以 **2 kHz**（`FOC_CALC_PERIOD = 2000 µs`）运行，由硬件 `esp_timer` 驱动：

1. **定时器 ISR** (`_timer_callback_static`)：每 2ms 触发一次，通过 FreeRTOS 直接任务通知发送信号给 `foc_calc_task`。
2. **foc_calc_task**（优先级 20，绑定到核心 0）：阻塞在 `ulTaskNotifyTake`，收到 ISR 通知后唤醒，执行一次控制迭代。

一次控制迭代的流水线（在 `_set_dq_out_exec` 中）：
1. 将 `Ud`/`Uq` 钳制到 `±FOC_MCPWM_OUTPUT_LIMIT`
2. **逆 Park 变换** (`foc_inverse_park_transform`)：旋转 `dq` 坐标系 → 静止 `αβ` 坐标系，使用电角度 `e_theta_rad`
3. **SVPWM 占空比计算** (`foc_svpwm_duty_calculate`)：`αβ` → 三相 PWM 占空比，通过 7 段式 SVPWM（6 扇区空间矢量调制）。被注释掉的 `foc_inverse_clarke_transform` 是 SPWM 替代方案。
4. **占空比 → 比较器值**：按 `FOC_MCPWM_PERIOD/4` 缩放和偏移，用于中心对齐 PWM，然后通过 `svpwm_inverter_set_duty` 写入 MCPWM 比较器。

### 关键文件

| 文件 | 职责 |
| --- | --- |
| `project_conf.h` | 硬件常量：GPIO 引脚（U/V/W PWM、使能），极对数（7），PWM 分辨率（80MHz），周期（2000 ticks） |
| `esp_foc.h/.cpp` | 纯数学：电角度计算、逆 Park、逆 Clarke、SVPWM 扇区与占空比计算 |
| `esp_svpwm.h/.cpp` | MCPWM 硬件抽象：创建/启动/停止逆变器，设置三相比较器值。封装 ESP-IDF `mcpwm_*` API |
| `motor_foc_driver.h/.cpp` | `FocDriver` 类：将逆变器、定时器和 FreeRTOS 任务整合为完整的 FOC 运行时 |
| `app_main.cpp` | 当前为占位代码 — 循环打印 "Hello world!" |

### SVPWM 逆变器 (`esp_svpwm`)

每个逆变器拥有一个 MCPWM 定时器 + 3 组 operator/comparator/generator（U、V、W 相），全部在 group 0。定时器以**递增-递减计数模式**（中心对齐 PWM）运行。比较事件在 `TIMER_DIRECTION_UP`（设置输出 LOW）和 `TIMER_DIRECTION_DOWN`（设置输出 HIGH）时均触发，产生对称的 PWM 波形。

### `FocDriver` 类

- 构造函数接收硬件引脚（`u_gpio`、`v_gpio`、`w_gpio`、`en_gpio`）和 `pole_pairs` — 全部从 `project_conf.h` 传入。
- `set_dq(float Ud, float Uq)` 是公共力矩控制接口（当前 .cpp 中实现为桩函数 — 控制循环 `_set_dq_out_loop()` 也仅阻塞等待通知而未执行计算；实际的 FOC 数学运算在 `_set_dq_out_exec` 中，但未被调用，说明项目处于活跃开发中）。
- 静态 C 回调（`_timer_callback_static`、`_foc_task_static`）通过 `void *args` / `this` 指针转发到成员函数。

### 重要 ESP-IDF 配置要求

- `CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD` 必须启用（menuconfig → Component config → ESP Timer），以支持 `esp_timer` ISR 回调。
- 定时器任务优先级应在 menuconfig 中调整（Component config → ESP Timer → Timer task priority），以确保 2kHz 循环满足实时截止时间。
- MCPWM 定时器使用 `MCPWM_TIMER_CLK_SRC_DEFAULT`（80 MHz PLL），分辨率 = 80 MHz，周期 = 2000 → 25 kHz PWM 载波频率。