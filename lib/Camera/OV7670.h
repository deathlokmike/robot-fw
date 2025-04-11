#pragma once
#include <Wire.h>

#include "I2SCamera.h"

struct RegisterValue {
    uint8_t reg;
    byte val;
};

class OV7670 : public I2SCamera {
   protected:
    TwoWire *_wire;
    uint8_t i2cAddress;
    bool QQVGARGB565();
    uint8_t writeRegister(uint8_t reg, uint8_t val);

    bool QQVGA();
    bool saturation();
    bool frameControl(int hStart, int hStop, int vStart, int vStop);
    bool writeRegisters(const RegisterValue regValues[], uint8_t count);

   public:
    bool reset();
    bool ping();
    OV7670(const uint8_t addr = 0x21) : _wire{&Wire}, i2cAddress{addr} {};
    bool init(const uint8_t VSYNC, const uint8_t HREF, const uint8_t XCLK,
              const uint8_t PCLK, const uint8_t D0, const uint8_t D1,
              const uint8_t D2, const uint8_t D3, const uint8_t D4,
              const uint8_t D5, const uint8_t D6, const uint8_t D7);

    // camera registers
    static constexpr uint8_t REG_GAIN{0x00};
    static constexpr uint8_t REG_BLUE{0x01};
    static constexpr uint8_t REG_RED{0x02};
    static constexpr uint8_t REG_COM1{0x04};
    static constexpr uint8_t REG_VREF{0x03};
    static constexpr uint8_t REG_COM4{0x0d};
    static constexpr uint8_t REG_COM5{0x0e};
    static constexpr uint8_t REG_COM6{0x0f};
    static constexpr uint8_t REG_AECH{0x10};
    static constexpr uint8_t REG_CLKRC{0x11};
    static constexpr uint8_t REG_COM7{0x12};
    static constexpr uint8_t COM7_RGB{0x04};
    static constexpr uint8_t REG_COM8{0x13};
    static constexpr uint8_t COM8_FASTAEC{0x80};  // Enable fast AGC/AEC
    static constexpr uint8_t COM8_AECSTEP{0x40};  // Unlimited AEC step size
    static constexpr uint8_t COM8_BFILT{0x20};    // Band filter enable
    static constexpr uint8_t COM8_AGC{0x04};      // Auto gain enable
    static constexpr uint8_t COM8_AWB{0x02};      // White balance enable
    static constexpr uint8_t COM8_AEC{0x0};
    static constexpr uint8_t REG_COM9{0x14};
    static constexpr uint8_t REG_COM10{0x15};
    static constexpr uint8_t REG_COM14{0x3E};
    static constexpr uint8_t REG_COM11{0x3B};
    static constexpr uint8_t COM11_NIGHT{0x80};
    static constexpr uint8_t COM11_NMFR{0x60};
    static constexpr uint8_t COM11_HZAUTO{0x10};
    static constexpr uint8_t COM11_50HZ{0x08};
    static constexpr uint8_t COM11_EXP{0x0};
    static constexpr uint8_t REG_TSLB{0x3A};
    static constexpr uint8_t REG_RGB444{0x8C};
    static constexpr uint8_t REG_COM15{0x40};
    static constexpr uint8_t COM15_RGB565{0x10};
    static constexpr uint8_t COM15_R00FF{0xc0};
    static constexpr uint8_t REG_HSTART{0x17};
    static constexpr uint8_t REG_HSTOP{0x18};
    static constexpr uint8_t REG_HREF{0x32};
    static constexpr uint8_t REG_VSTART{0x19};
    static constexpr uint8_t REG_VSTOP{0x1A};
    static constexpr uint8_t REG_COM3{0x0C};
    static constexpr uint8_t REG_MVFP{0x1E};
    static constexpr uint8_t REG_COM13{0x3d};
    static constexpr uint8_t COM13_UVSAT{0x40};
    static constexpr uint8_t REG_SCALING_XSC{0x70};
    static constexpr uint8_t REG_SCALING_YSC{0x71};
    static constexpr uint8_t REG_SCALING_DCWCTR{0x72};
    static constexpr uint8_t REG_SCALING_PCLK_DIV{0x73};
    static constexpr uint8_t REG_SCALING_PCLK_DELAY{0xa2};
    static constexpr uint8_t REG_BD50MAX{0xa5};
    static constexpr uint8_t REG_BD60MAX{0xab};
    static constexpr uint8_t REG_AEW{0x24};
    static constexpr uint8_t REG_AEB{0x25};
    static constexpr uint8_t REG_VPT{0x26};
    static constexpr uint8_t REG_HAECC1{0x9f};
    static constexpr uint8_t REG_HAECC2{0xa0};
    static constexpr uint8_t REG_HAECC3{0xa6};
    static constexpr uint8_t REG_HAECC4{0xa7};
    static constexpr uint8_t REG_HAECC5{0xa8};
    static constexpr uint8_t REG_HAECC6{0xa9};
    static constexpr uint8_t REG_HAECC7{0xaa};
    static constexpr uint8_t REG_COM12{0x3c};
    static constexpr uint8_t REG_GFIX{0x69};
    static constexpr uint8_t REG_COM16{0x41};
    static constexpr uint8_t COM16_AWBGAIN{0x08};
    static constexpr uint8_t REG_EDGE{0x3f};
    static constexpr uint8_t REG_REG76{0x76};
    static constexpr uint8_t ADCCTR0{0x20};
};
