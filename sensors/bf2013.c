/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for
 * details.
 *
 * bf2013 driver.
 *
 */
#include "bf2013.h"
#include "bf2013_regs.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sccb.h"
#include "xclk.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "bf2013";
#endif
static const uint8_t default_regs[][2] = {
    {0x12, 0x80}, // COM7 Resets all registers to default values
    {0x09, 0x03}, // COM2 0x20==1 4x data out drive capability
    {0x15, 0x02}, // COM10  HSYNC:active high, VSYNC:active high
    {0x3a, 0x00}, // TSLB   IF YUV422 IS selected YUYV else RGB565 R5G3H,G3LB5
    {0x12, 0x00}, // COM7
    {0x1e, 0x00}, {0x13, 0x00}, {0x01, 0x14}, {0x02, 0x21}, {0x8c, 0x01},
    {0x8d, 0xcb}, {0x87, 0x20}, {0x1b, 0x80}, {0x11, 0x80}, {0x2b, 0x20},
    {0x92, 0x40}, {0x06, 0xe0}, {0x29, 0x54}, {0xeb, 0x30}, {0xbb, 0x20},
    {0xf5, 0x21}, {0xe1, 0x3c}, {0x16, 0x01}, {0xe0, 0x0b}, {0x2f, 0xf6},
    {0x1f, 0x20}, {0x22, 0x20}, {0x26, 0x20}, {0x33, 0x20}, {0x34, 0x08},
    {0x35, 0x50}, {0x65, 0x4a}, {0x66, 0x48}, {0x36, 0x05}, {0x37, 0xf6},
    {0x38, 0x46}, {0x9b, 0xf6}, {0x9c, 0x46}, {0xbc, 0x01}, {0xbd, 0xf6},
    {0xbe, 0x46}, {0x70, 0x6f}, {0x72, 0x3f}, {0x73, 0x3f}, {0x74, 0x27},
    {0x77, 0x90}, {0x79, 0x48}, {0x7a, 0x1e}, {0x7b, 0x30}, {0x24, 0x70},
    {0x25, 0x80}, {0x80, 0x55}, {0x81, 0x02}, {0x82, 0x14}, {0x83, 0x23},
    {0x9a, 0x23}, {0x84, 0x1a}, {0x85, 0x20}, {0x86, 0x30}, {0x89, 0x02},
    {0x8a, 0x64}, {0x8b, 0x02}, {0x8e, 0x07}, {0x8f, 0x79}, {0x94, 0x0a},
    {0x96, 0xa6}, {0x97, 0x0c}, {0x98, 0x18}, {0x9d, 0x93}, {0x9e, 0x7a},
    {0x3b, 0x60}, {0x3c, 0x20}, {0x39, 0x80}, {0x3f, 0xb0}, {0x40, 0x9b},
    {0x41, 0x88}, {0x42, 0x6e}, {0x43, 0x59}, {0x44, 0x4d}, {0x45, 0x45},
    {0x46, 0x3e}, {0x47, 0x39}, {0x48, 0x35}, {0x49, 0x31}, {0x4b, 0x2e},
    {0x4c, 0x2b}, {0x4e, 0x26}, {0x4f, 0x22}, {0x50, 0x1f}, {0x51, 0x05},
    {0x52, 0x10}, {0x53, 0x0b}, {0x54, 0x15}, {0x57, 0x87}, {0x58, 0x72},
    {0x59, 0x5f}, {0x5a, 0x7e}, {0x5b, 0x1f}, {0x5c, 0x0e}, {0x5d, 0x95},
    {0x60, 0x24}, {0x6a, 0x01}, {0x23, 0x66}, {0xa0, 0x03}, {0xa1, 0x31},
    {0xa2, 0x0e}, {0xa3, 0x27}, {0xa4, 0x08}, {0xa5, 0x25}, {0xa6, 0x06},
    {0xa7, 0x80}, {0xa8, 0x7e}, {0xa9, 0x20}, {0xaa, 0x20}, {0xab, 0x20},
    {0xac, 0x3c}, {0xad, 0xf0}, {0xae, 0x80}, {0xaf, 0x00}, {0xc5, 0x18},
    {0xc6, 0x00}, {0xc7, 0x20}, {0xc8, 0x18}, {0xc9, 0x20}, {0xca, 0x17},
    {0xcb, 0x1f}, {0xcc, 0x40}, {0xcd, 0x58}, {0xee, 0x4c}, {0xb0, 0xe0},
    {0xb1, 0xc0}, {0xb2, 0xb0}, {0xb3, 0x88}, {0x56, 0x40}, {0x13, 0x07}};

static int get_reg(sensor_t *sensor, int reg, int mask) {
  int ret = SCCB_Read(sensor->slv_addr, reg & 0xFF);
  if (ret > 0) {
    ret &= mask;
  }
  return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value) {
  int ret = 0;
  ret = SCCB_Read(sensor->slv_addr, reg & 0xFF);
  if (ret < 0) {
    return ret;
  }
  value = (ret & ~mask) | (value & mask);
  ret = SCCB_Write(sensor->slv_addr, reg & 0xFF, value);
  return ret;
}

static int set_reg_bits(sensor_t *sensor, uint8_t reg, uint8_t offset,
                        uint8_t length, uint8_t value) {
  int ret = 0;
  ret = SCCB_Read(sensor->slv_addr, reg);
  if (ret < 0) {
    return ret;
  }
  uint8_t mask = ((1 << length) - 1) << offset;
  value = (ret & ~mask) | ((value << offset) & mask);
  ret = SCCB_Write(sensor->slv_addr, reg & 0xFF, value);
  return ret;
}

static int get_reg_bits(sensor_t *sensor, uint8_t reg, uint8_t offset,
                        uint8_t length) {
  int ret = 0;
  ret = SCCB_Read(sensor->slv_addr, reg);
  if (ret < 0) {
    return ret;
  }
  uint8_t mask = ((1 << length) - 1) << offset;
  return (ret & mask) >> offset;
}

static int reset(sensor_t *sensor) {
  int i = 0;
  const uint8_t(*regs)[2];

  // Reset all registers

  //   int ret = SCCB_Write(sensor->slv_addr, COM7, COM7_RESET);
  int ret = SCCB_Write(sensor->slv_addr, 0x09, 0x03);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "bf2013 reset failure.");
  }

  // Delay 10 ms
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // Write default regsiters
  uint8_t value = 0;
  for (i = 1, regs = default_regs; i < sizeof(default_regs) / 2; i++) {
    SCCB_Write(sensor->slv_addr, regs[i][0], regs[i][1]);
    value = 0;
    value = SCCB_Read(sensor->slv_addr, regs[i][0]);
    if (value != regs[i][1]) {
      ESP_LOGE(TAG, "regs:[%02X] write: %02X,read:%02X", regs[i][0], regs[i][1],
               value);
    } else {
      ESP_LOGI(TAG, "regs:[%02X] write: %02X,read:%02X", regs[i][0], regs[i][1],
               value);
    }

    // ESP_LOGI(TAG, "regs:[%02X],read:%02X", regs[i][0],
    //          SCCB_Read(sensor->slv_addr, regs[i][0]));
  }

  // Delay
  vTaskDelay(30 / portTICK_PERIOD_MS);

  return 0;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat) {
  int ret = 0;
  sensor->pixformat = pixformat;
  switch (pixformat) {
  case PIXFORMAT_RGB565:
    set_reg_bits(sensor, COM7, 2, 1, 1);
    break;
  case PIXFORMAT_RAW:
    set_reg_bits(sensor, COM7, 0, 3, 0x4);
    break;
  case PIXFORMAT_YUV422:
  case PIXFORMAT_GRAYSCALE:
    set_reg_bits(sensor, 0x12, 2, 1, 0);
    break;
  default:
    return -1;
  }
  // Delay
  vTaskDelay(30 / portTICK_PERIOD_MS);

  return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize) {
  int ret = 0;
  if (framesize > FRAMESIZE_VGA) {
    return -1;
  }
  uint16_t w = resolution[framesize].width;
  uint16_t h = resolution[framesize].height;

  sensor->status.framesize = framesize;

  // Write MSBs
  ret |= SCCB_Write(sensor->slv_addr, HSTART, 0);
  ret |= SCCB_Write(sensor->slv_addr, HSTOP, w >> 2);

  ret |= SCCB_Write(sensor->slv_addr, VSTART, 0);
  ret |= SCCB_Write(sensor->slv_addr, VSTOP, h >> 2);

  // Write LSBs
  ret |= SCCB_Write(sensor->slv_addr, VHREF, 0);
  printf("%s %d\r\n", __func__, __LINE__);
  if ((w <= 320) && (h <= 240)) {
    printf("%s %d\r\n", __func__, __LINE__);
    // Enable auto-scaling/zooming factors
    // ret |= SCCB_Write(sensor->slv_addr, 0x12, 0x50);
    set_reg_bits(sensor, 0x12, 4, 1, 1);

    ret |= SCCB_Write(sensor->slv_addr, HSTART, (80 - w / 4));
    ret |= SCCB_Write(sensor->slv_addr, HSTOP, (80 + w / 4));

    ret |= SCCB_Write(sensor->slv_addr, VSTART, (60 - h / 4));
    ret |= SCCB_Write(sensor->slv_addr, VSTOP, (60 + h / 4));

    ret |= SCCB_Write(sensor->slv_addr, VHREF, 0);

  } else if ((w <= 640) && (h <= 480)) {
    // Enable auto-scaling/zooming factors
    // ret |= SCCB_Write(sensor->slv_addr, 0x12, 0x40);
    set_reg_bits(sensor, 0x12, 4, 1, 0);

    ret |= SCCB_Write(sensor->slv_addr, HSTART, (80 - w / 8));
    ret |= SCCB_Write(sensor->slv_addr, HSTOP, (80 + w / 8));

    ret |= SCCB_Write(sensor->slv_addr, VSTART, (60 - h / 8));
    ret |= SCCB_Write(sensor->slv_addr, VSTOP, (60 + h / 8));

    ret |= SCCB_Write(sensor->slv_addr, VHREF, 0);
  }
  // Delay
  vTaskDelay(30 / portTICK_PERIOD_MS);

  return ret;
}

static int set_colorbar(sensor_t *sensor, int value) {
  int ret = 0;
  sensor->status.colorbar = value;

  ret |= SCCB_Write(sensor->slv_addr, TEST_MODE, value);

  return ret;
}

static int set_whitebal(sensor_t *sensor, int enable) {
  if (set_reg_bits(sensor, COM8, 1, 1, enable) >= 0) {
    sensor->status.awb = !!enable;
  }
  return sensor->status.awb;
}

static int set_gain_ctrl(sensor_t *sensor, int enable) {
  if (set_reg_bits(sensor, COM8, 2, 1, enable) >= 0) {
    sensor->status.agc = !!enable;
  }
  return sensor->status.agc;
}

static int set_exposure_ctrl(sensor_t *sensor, int enable) {
  if (set_reg_bits(sensor, COM8, 0, 1, enable) >= 0) {
    sensor->status.aec = !!enable;
  }
  return sensor->status.aec;
}

static int set_hmirror(sensor_t *sensor, int enable) {
  if (set_reg_bits(sensor, MVFP, 5, 1, enable) >= 0) {
    sensor->status.hmirror = !!enable;
  }
  return sensor->status.hmirror;
}

static int set_vflip(sensor_t *sensor, int enable) {
  if (set_reg_bits(sensor, MVFP, 4, 1, enable) >= 0) {
    sensor->status.vflip = !!enable;
  }
  return sensor->status.vflip;
}

static int set_raw_gma_dsp(sensor_t *sensor, int enable) {
  int ret = 0;
  ret = set_reg_bits(sensor, 0xf1, 1, 1, !enable);
  if (ret == 0) {
    ESP_LOGD(TAG, "Set raw_gma to: %d", !enable);
    sensor->status.raw_gma = !enable;
  }
  return ret;
}

static int set_lenc_dsp(sensor_t *sensor, int enable) {
  int ret = 0;
  ret = set_reg_bits(sensor, 0xf1, 0, 1, !enable);
  if (ret == 0) {
    ESP_LOGD(TAG, "Set lenc to: %d", !enable);
    sensor->status.lenc = !enable;
  }
  return ret;
}

static int set_agc_gain(sensor_t *sensor, int option) {
  int ret = 0;
  ret = set_reg_bits(sensor, 0x13, 4, 1, !!option);
  if (ret == 0) {
    ESP_LOGD(TAG, "Set gain to: %d", !!option);
    sensor->status.agc_gain = !!option;
  }
  return ret;
}

static int set_awb_gain_dsp(sensor_t *sensor, int value) {
  int ret = 0;
  ret = SCCB_Write(sensor->slv_addr, 0xa6, value);
  if (ret == 0) {
    ESP_LOGD(TAG, "Set awb gain threthold to: %d", value);
    sensor->status.awb_gain = value;
  }
  return ret;
}

static int set_brightness(sensor_t *sensor, int level) {
  int ret = 0;
  ret = SCCB_Write(sensor->slv_addr, 0x55, level);
  if (ret == 0) {
    ESP_LOGD(TAG, "Set brightness to: %d", level);
    sensor->status.brightness = level;
  }
  return ret;
}

static int set_contrast(sensor_t *sensor, int level) {
  int ret = 0;
  ret = SCCB_Write(sensor->slv_addr, 0x56, level);
  if (ret == 0) {
    ESP_LOGD(TAG, "Set contrast to: %d", level);
    sensor->status.contrast = level;
  }
  return ret;
}

static int set_sharpness(sensor_t *sensor, int level) {
  int ret = 0;
  ret = SCCB_Write(sensor->slv_addr, INTCTR, level);
  if (ret == 0) {
    ESP_LOGD(TAG, "Set sharpness to: %d", level);
    sensor->status.sharpness = level;
  }
  return ret;
}

static int init_status(sensor_t *sensor) {
  sensor->status.brightness = SCCB_Read(sensor->slv_addr, 0x55);
  sensor->status.contrast = SCCB_Read(sensor->slv_addr, 0x56);
  sensor->status.saturation = 0;
  sensor->status.ae_level = 0;

  sensor->status.gainceiling = SCCB_Read(sensor->slv_addr, 0x87);
  sensor->status.awb = get_reg_bits(sensor, 0x13, 1, 1);
  sensor->status.awb_gain = SCCB_Read(sensor->slv_addr, 0xa6);
  sensor->status.aec = get_reg_bits(sensor, 0x13, 0, 1);

  sensor->status.agc = get_reg_bits(sensor, 0x13, 2, 1);

  sensor->status.raw_gma = get_reg_bits(sensor, 0xf1, 1, 1);
  sensor->status.lenc = get_reg_bits(sensor, 0xf1, 0, 1);
  sensor->status.hmirror = get_reg_bits(sensor, 0x1e, 5, 1);
  sensor->status.vflip = get_reg_bits(sensor, 0x1e, 4, 1);

  sensor->status.colorbar = SCCB_Read(sensor->slv_addr, 0xb9);
  sensor->status.sharpness = SCCB_Read(sensor->slv_addr, 0x70);

  int i = 0;
  const uint8_t(*regs)[2];

  // Write default regsiters
  for (i = 0, regs = default_regs; i < sizeof(default_regs) / 2; i++) {
    // SCCB_Write(sensor->slv_addr, regs[i][0], regs[i][1]);
    // ESP_LOGI(TAG, "regs:[%02X] write: %02X,read:%02X", regs[i][0],
    // regs[i][1],
    //  SCCB_Read(sensor->slv_addr, regs[i][0]));
    ESP_LOGI(TAG, "regs:[%02X],read:%02X", regs[i][0],
             SCCB_Read(sensor->slv_addr, regs[i][0]));
  }

  return 0;
}

static int set_dummy(sensor_t *sensor, int val) { return -1; }
static int set_gainceiling_dummy(sensor_t *sensor, gainceiling_t val) {
  return -1;
}
static int set_res_raw(sensor_t *sensor, int startX, int startY, int endX,
                       int endY, int offsetX, int offsetY, int totalX,
                       int totalY, int outputX, int outputY, bool scale,
                       bool binning) {
  return -1;
}
static int _set_pll(sensor_t *sensor, int bypass, int multiplier, int sys_div,
                    int root_2x, int pre_div, int seld5, int pclk_manual,
                    int pclk_div) {
  return -1;
}

static int set_xclk(sensor_t *sensor, int timer, int xclk) {
  int ret = 0;
  sensor->xclk_freq_hz = xclk * 1000000U;
  ret = xclk_timer_conf(timer, sensor->xclk_freq_hz);
  return ret;
}

int bf2013_detect(int slv_addr, sensor_id_t *id) {
  if (BF2013_SCCB_ADDR == slv_addr) {
    uint16_t PID = SCCB_Read(slv_addr, REG_PID);
    ESP_LOGI(TAG, "PID=0x%x", PID);
    if (BF2013_PID == PID) {
      id->PID = PID;
      id->VER = SCCB_Read(slv_addr, REG_VER);
      id->MIDL = SCCB_Read(slv_addr, REG_MIDL);
      id->MIDH = SCCB_Read(slv_addr, REG_MIDH);
      return PID;
    } else {
      ESP_LOGI(TAG, "Mismatch PID=0x%x", PID);
    }
  }
  return 0;
}

int bf2013_init(sensor_t *sensor) {
  // Set function pointers
  sensor->reset = reset;
  sensor->init_status = init_status;
  sensor->set_pixformat = set_pixformat;
  sensor->set_framesize = set_framesize;
  sensor->set_brightness = set_brightness;
  sensor->set_contrast = set_contrast;

  sensor->set_colorbar = set_colorbar;

  sensor->set_gain_ctrl = set_gain_ctrl;
  sensor->set_exposure_ctrl = set_exposure_ctrl;
  sensor->set_hmirror = set_hmirror;
  sensor->set_vflip = set_vflip;

  sensor->set_whitebal = set_whitebal;

  sensor->set_awb_gain = set_awb_gain_dsp;
  sensor->set_agc_gain = set_agc_gain;

  sensor->set_raw_gma = set_raw_gma_dsp;
  sensor->set_lenc = set_lenc_dsp;

  sensor->set_sharpness = set_sharpness;
  // not supported
  sensor->set_saturation = set_dummy;
  sensor->set_denoise = set_dummy;
  sensor->set_quality = set_dummy;
  sensor->set_special_effect = set_dummy;
  sensor->set_wb_mode = set_dummy;
  sensor->set_ae_level = set_dummy;
  sensor->set_gainceiling = set_gainceiling_dummy;

  sensor->get_reg = get_reg;
  sensor->set_reg = set_reg;
  sensor->set_res_raw = set_res_raw;
  sensor->set_pll = _set_pll;
  sensor->set_xclk = set_xclk;

  // Retrieve sensor's signature
  sensor->id.MIDH = SCCB_Read(sensor->slv_addr, REG_MIDH);
  sensor->id.MIDL = SCCB_Read(sensor->slv_addr, REG_MIDL);
  sensor->id.PID = SCCB_Read(sensor->slv_addr, REG_PID);
  sensor->id.VER = SCCB_Read(sensor->slv_addr, REG_VER);

  ESP_LOGD(TAG, "bf2013 Attached");

  return 0;
}
