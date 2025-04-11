#pragma once

#include "DMABuffer.h"
#include "XClk.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"

enum class i2s_sampling_mode_t : uint8_t {
    SM_0A0B_0B0C = 0,
    SM_0A0B_0C0D = 1,
    SM_0A00_0B00 = 3
};

class I2SCamera {
   public:
    static gpio_num_t vSyncPin;
    static int blocksReceived;
    static int framesReceived;
    static int xres;
    static int yres;
    static intr_handle_t i2sInterruptHandle;
    static intr_handle_t vSyncInterruptHandle;
    static int dmaBufferCount;
    static int dmaBufferActive;
    static DMABuffer **dmaBuffer;
    static unsigned char *frame;
    static int framePointer;
    static int frameBytes;
    static volatile bool stopSignal;

    static inline void i2sConfReset() {
        const uint32_t lc_conf_reset_flags =
            I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
        I2S0.lc_conf.val |= lc_conf_reset_flags;
        I2S0.lc_conf.val &= ~lc_conf_reset_flags;

        const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M |
                                          I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
        I2S0.conf.val |= conf_reset_flags;
        I2S0.conf.val &= ~conf_reset_flags;
        while (I2S0.state.rx_fifo_reset_back);
    }

    void start() { i2sRun(); }

    void stop() {
        stopSignal = true;
        while (stopSignal);
    }

    void oneFrame() {
        start();
        stop();
    }

    static void i2sStop();
    static void i2sRun();

    static void dmaBufferInit(int bytes);
    static void dmaBufferDeinit();

    static bool initVSync(int pin);
    static void deinitVSync();

    static void IRAM_ATTR i2sInterrupt(void *arg);
    static void IRAM_ATTR vSyncInterrupt(void *arg);

    static bool i2sInit(const int VSYNC, const int HREF, const int PCLK,
                        const int D0, const int D1, const int D2, const int D3,
                        const int D4, const int D5, const int D6, const int D7);

    static bool init(const int XRES, const int YRES, const int VSYNC,
                     const int HREF, const int XCLK, const int PCLK,
                     const int D0, const int D1, const int D2, const int D3,
                     const int D4, const int D5, const int D6, const int D7);
};
