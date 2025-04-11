#include "OV7670.h"

bool OV7670::init(const uint8_t VSYNC, const uint8_t HREF, const uint8_t XCLK,
                  const uint8_t PCLK, const uint8_t D0, const uint8_t D1,
                  const uint8_t D2, const uint8_t D3, const uint8_t D4,
                  const uint8_t D5, const uint8_t D6, const uint8_t D7) {
    ESP_LOGD(cameraLogTag, "Start init camera");
    if (!ClockEnable(XCLK, 10000000)) return false;
    if (!I2SCamera::init(160, 120, VSYNC, HREF, XCLK, PCLK, D0, D1, D2, D3, D4,
                         D5, D6, D7))
        return false;

    ESP_LOGD(cameraLogTag, "Waiting for VSYNC...");
    pinMode(VSYNC, INPUT);
    while (!digitalRead(VSYNC));
    while (digitalRead(VSYNC));
    ESP_LOGD(cameraLogTag, "Done");
    if (!ping()) return false;
    return QQVGARGB565();
}

bool OV7670::QQVGARGB565() {
    ESP_LOGD(cameraLogTag, "Start QQVGARGB565");
    if (!reset()) return false;
    if (!QQVGA()) return false;
    if (!frameControl(196, 52, 8, 488)) return false;
    if (!saturation()) return false;
    ESP_LOGD(cameraLogTag, "Done QQVGARGB565");
    return true;
}

bool OV7670::reset() {
    ESP_LOGD(cameraLogTag, "Reset");
    const RegisterValue regValues[] = {
        {REG_COM7, 0b10000000},   // Reset
        {REG_COM7, 0b00000000},   // Fix error 2
        {REG_CLKRC, 0b10000000},  // Enable double clock
        {REG_COM11, 0b00001010},  // Auto 50/60Hz detect + exposure
        {REG_COM7, 0b00000100},   // Select RGB
        {REG_COM15, 0b11010000}   // RGB565
    };
    return writeRegisters(regValues, 6);
}

bool OV7670::QQVGA() {
    ESP_LOGD(cameraLogTag, "QQVGA");
    const RegisterValue regValues[] = {
        // 160x120 (1/4)
        // writeRegister(REG_CLKRC, 0x01);
        // DCW enable
        {REG_COM3, 0x04},
        // pixel clock divided by 4, manual scaling enable,
        // DCW and PCLK controlled by register
        {REG_COM14, 0x1a},
        {REG_SCALING_XSC, 0x3a},
        {REG_SCALING_YSC, 0x35},
        // downsample by 4
        {REG_SCALING_DCWCTR, 0x22},
        // pixel clock divided by 4
        {REG_SCALING_PCLK_DIV, 0xf2},
        {REG_SCALING_PCLK_DELAY, 0x02}};
    return writeRegisters(regValues, 7);
}

bool OV7670::saturation() {
    // color matrix values
    const RegisterValue regValues[] = {
        {0xb0, 0x84},
        {0x4f, 0x80},
        {0x50, 0x80},
        {0x51, 0x00},
        {0x52, 0x22},
        {0x53, 0x5e},
        {0x54, 0x80},
        // matrix signs
        {0x58, 0x9e},
        // AWB on
        {0x13, 0xe7},
        // Simple AWB
        {0x6f, 0x9f},
    };
    return writeRegisters(regValues, 10);
}

bool OV7670::frameControl(int hStart, int hStop, int vStart, int vStop) {
    const RegisterValue regValues[] = {
        {REG_HSTART, static_cast<byte>(hStart >> 3)},
        {REG_HSTOP, static_cast<byte>(hStop >> 3)},
        {REG_HREF,
         static_cast<byte>(((hStop & 0b111) << 3) | (hStart & 0b111))},
        {REG_VSTART, static_cast<byte>(vStart >> 2)},
        {REG_VSTOP, static_cast<byte>(vStop >> 2)},
        {REG_VREF, static_cast<byte>(((vStop & 0b11) << 2) | (vStart & 0b11))}};
    return writeRegisters(regValues, 6);
}

bool OV7670::writeRegisters(const RegisterValue regValues[], uint8_t count) {
    for (size_t i = 0; i < count; ++i) {
        uint8_t i2cCode = writeRegister(regValues[i].reg, regValues[i].val);
        if (i2cCode != 0) {
            ESP_LOGE(cameraLogTag, "Error: %d, Reg: 0x%X, Val: 0x%X", i2cCode,
                     regValues[i].reg, regValues[i].val);
            return false;
        } else {
            ESP_LOGD(cameraLogTag, "Successful: %d, Reg: 0x%X, Val: 0x%X",
                     i2cCode, regValues[i].reg, regValues[i].val);
        }
    }
    return true;
}

uint8_t OV7670::writeRegister(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission();
}

bool OV7670::ping() {
    ESP_LOGD(cameraLogTag, "Ping 0x%X", i2cAddress);
    _wire->beginTransmission(i2cAddress);
    uint8_t error = _wire->endTransmission();
    if (error == 0) {
        return true;
    }
    return false;
}