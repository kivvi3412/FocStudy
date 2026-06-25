//
// Created by HAIRONG ZHU on 25-1-14.
//

#include "iic_as5600.h"
#include "project_conf.h"
#include <cmath>

static const char *TAG = "AS5600";

AS5600::AS5600(i2c_master_bus_handle_t bus_handle, uint8_t device_address) {
  if (bus_handle == nullptr) {
    ESP_LOGE(TAG, "I2C master bus not initialized");
    return;
  }
  // 配置 I2C 设备
  i2c_device_config_t dev_config = {};
  dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_config.device_address = device_address;
  dev_config.scl_speed_hz = IIC_MASTER_FREQ_HZ;
  esp_err_t ret =
      i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
  }
}

uint16_t AS5600::_location_read_raw() {
  uint8_t reg = IIC_AS5600_RAW_ANGLE_REG;
  uint8_t buffer[2] = {0};

  esp_err_t ret =
      i2c_master_transmit_receive(dev_handle_, &reg, 1, buffer, 2, -1);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C transmit/receive failed: %s", esp_err_to_name(ret));
    return ret;
  }

  return ((uint16_t)buffer[0] << 8) | buffer[1];
}

float AS5600::read_radian_from_sensor() {
  auto current_radian = float(_location_read_raw() * M_TWOPI /
                              IIC_AS5600_RESOLUTION); // 读取传感器的弧度
  int64_t now = esp_timer_get_time();
  _update_total_radian_and_velocity(current_radian,
                                    now); // 更新累计的总弧度和转速
  return current_radian;
}

float AS5600::read_radian_from_sensor_with_no_update() {
  auto current_radian =
      float(_location_read_raw() * M_TWOPI / IIC_AS5600_RESOLUTION);
  return current_radian;
}

esp_err_t AS5600::_update_total_radian_and_velocity(float currentRadian,
                                                    int64_t now_us) {
  float deltaRadian = currentRadian - previous_radian_;
  if (fabsf(deltaRadian) > M_PI) {
    if (deltaRadian > 0) {
      deltaRadian -= M_TWOPI;
    } else {
      deltaRadian += M_TWOPI;
    }
  }
  total_accumulated_radian_ += deltaRadian;
  previous_radian_ = currentRadian;

  // 使用实际时间戳计算速度（取代固定 FOC_CALC_PERIOD）
  float dt_s = 0;
  if (prev_read_time_us_ > 0) {
    dt_s = (float)(now_us - prev_read_time_us_) * 1e-6f;
  }
  prev_read_time_us_ = now_us;

  if (dt_s > 1e-6f) { // 防止除零
    velocity_ = deltaRadian / dt_s;
  }

  // 低通滤波
  float alpha = FOC_LOW_PASS_FILTER_ALPHA;
  velocity_filter_ =
      alpha * velocity_ + (1 - alpha) * velocity_filter_; // 一阶低通滤波

  return ESP_OK;
}

void AS5600::set_custom_total_radian(float radian) { // 设置相对的累计弧度，将当前总累计弧度作为新的偏移
                                                     // (用户自定义
  relative_offset_radian_ = total_accumulated_radian_ - radian;
}

void AS5600::reset_custom_total_radian() { // 重置相对的累计弧度，将当前总累计弧度作为新的偏移
                                           // (用户自定义
  relative_offset_radian_ = total_accumulated_radian_;
}

float AS5600::get_radian() const { return previous_radian_; }

float AS5600::get_total_radian() const { return total_accumulated_radian_; }

float AS5600::get_custom_total_radian() const {
  return total_accumulated_radian_ - relative_offset_radian_;
}

float AS5600::get_velocity() const { return velocity_; }

float AS5600::get_velocity_filter() const { return velocity_filter_; }

// ===================== 连续读取任务实现 =====================

void AS5600::start_continuous_read() {
  if (reader_running_) {
    ESP_LOGW(TAG, "Continuous reader already running");
    return;
  }
  reader_running_ = true;

  // 初始化缓存：先做一次同步读取
  cached_radian_ = read_radian_from_sensor_with_no_update();
  cached_timestamp_us_ = esp_timer_get_time();
  prev_read_time_us_ = cached_timestamp_us_;

  // 在 Core 1 上创建连续读取任务，优先级略低于 FOC 任务
  xTaskCreatePinnedToCore(_continuous_read_task_static, "as5600_reader", 2048,
                          this,
                          15, // 优先级 15（FOC 任务优先级 20）
                          &reader_task_handle_,
                          1 // Core 1（FOC 在 Core 0）
  );
  ESP_LOGI(TAG, "Continuous reader task started on Core 1");
}

void AS5600::stop_continuous_read() {
  if (!reader_running_) {
    return;
  }
  reader_running_ = false;

  // 等待任务自行退出
  if (reader_task_handle_ != nullptr) {
    // 给任务一点时间检测标志并退出
    vTaskDelay(pdMS_TO_TICKS(10));
    // 如果任务还没退出，强制删除
    if (eTaskGetState(reader_task_handle_) != eDeleted) {
      vTaskDelete(reader_task_handle_);
    }
    reader_task_handle_ = nullptr;
  }
  ESP_LOGI(TAG, "Continuous reader task stopped");
}

void AS5600::get_cached_angle_us(float *radian_out, int64_t *timestamp_us_out) {
  // 使用自旋锁保证 radian + timestamp 的原子性读取
  taskENTER_CRITICAL(&spinlock_);
  *radian_out = cached_radian_;
  *timestamp_us_out = cached_timestamp_us_;
  taskEXIT_CRITICAL(&spinlock_);
}

void AS5600::_continuous_read_task_static(void *arg) {
  auto *self = static_cast<AS5600 *>(arg);
  self->_continuous_read_task();
}

void AS5600::_continuous_read_task() {
  ESP_LOGI(TAG, "Continuous reader running on Core %d", xPortGetCoreID());

  while (reader_running_) {
    // 阻塞式 I2C 读取（~100µs @400kHz）
    uint16_t raw = _location_read_raw();
    int64_t now = esp_timer_get_time();
    auto radian = float(raw * M_TWOPI / IIC_AS5600_RESOLUTION);

    // 用自旋锁写入缓存（临界区极短，~100ns）
    taskENTER_CRITICAL(&spinlock_);
    cached_radian_ = radian;
    cached_timestamp_us_ = now;
    taskEXIT_CRITICAL(&spinlock_);

    // 更新速度和累计角度（这些是 float 原子写入，Core 0 读取是安全的）
    _update_total_radian_and_velocity(radian, now);
  }

  // 任务退出前清理
  reader_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}
