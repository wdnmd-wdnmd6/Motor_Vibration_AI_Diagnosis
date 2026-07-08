/*
 * iis3dwb.c - IIS3DWB 3-axis accelerometer SPI driver
 * CS=PA0, shared SPI2 bus with ST7789 LCD
 */

#include "iis3dwb.h"
#include "hardware.h"

/* --- Low-level SPI helpers (sensor CS toggled per transaction) --- */

static uint8_t _reg_read(uint8_t addr) {
    uint8_t val;
    IIS3DWB_CS_Low();
    SPI2_ReadWriteByte(addr | 0x80);
    val = SPI2_ReadWriteByte(0x00);
    IIS3DWB_CS_High();
    return val;
}

static void _reg_write(uint8_t addr, uint8_t data) {
    IIS3DWB_CS_Low();
    SPI2_ReadWriteByte(addr & 0x7F);
    SPI2_ReadWriteByte(data);
    IIS3DWB_CS_High();
}

static void _reg_read_multi(uint8_t addr, uint8_t* buf, uint8_t len) {
    IIS3DWB_CS_Low();
    SPI2_ReadWriteByte(addr | 0x80 | 0x40);  /* read + auto-increment */
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI2_ReadWriteByte(0x00);
    }
    IIS3DWB_CS_High();
}

/* --- Public API --- */

int IIS3DWB_Init(void) {
    uint8_t who;

    /* SPI2 is already configured by SPI2_Shared_Init() in LCD_Init */

    /* Read device ID */
    who = _reg_read(IIS3DWB_WHO_AM_I);
    if (who != IIS3DWB_DEVICE_ID) {
        return -1;
    }

    /* Restore default config (software reset) */
    _reg_write(IIS3DWB_CTRL3_C, 0x01);  /* SW_RESET */
    for (volatile int i = 0; i < 10000; i++) __NOP();  /* wait */

    /* CTRL3_C: BDU=1 (block-data-update), IF_INC=1 (auto-increment) */
    _reg_write(IIS3DWB_CTRL3_C, 0x44);

    /* CTRL1_XL: ODR=6.66kHz (1011), FS=+/-16g (11) */
    _reg_write(IIS3DWB_CTRL1_XL, 0xA0 | 0x0C  /* ODR=6.66kHz FS=16g */);

    /* CTRL6_C: BW=800Hz (011), low-noise mode */
    _reg_write(IIS3DWB_CTRL6_C, 0x03);
    _reg_write(0x18, 0x38); /* CTRL9_XL: Xen=1 Yen=1 Zen=1 */
    _reg_write(0x19, 0x38); /* try 0x19 as well */
    /* Readback debug */
    {
        uint8_t r6 = _reg_read(IIS3DWB_CTRL6_C);
        uint8_t r9a = _reg_read(0x18);
        uint8_t r9b = _reg_read(0x19);
        extern void uart1_send(const char*);
        char _b[40]; snprintf(_b,40,"CTRL6=%02X R18=%02X R19=%02X\r\n",r6,r9a,r9b);
        uart1_send(_b);
    }

        /* Debug: read output registers directly */
    {
        uint8_t r[6];
        _reg_read_multi(IIS3DWB_OUTX_L_A, r, 6);
        extern void uart1_send(const char*);
        char _b[64];
        snprintf(_b,64,"OUT: XL=%02X XH=%02X YL=%02X YH=%02X ZL=%02X ZH=%02X\r\n",
                 r[0],r[1],r[2],r[3],r[4],r[5]);
        uart1_send(_b);
    }
    return 0;
}

int IIS3DWB_DataReady(void) {
    return (_reg_read(IIS3DWB_STATUS_REG) & 0x01) ? 1 : 0;
}

int IIS3DWB_ReadSample(iis3dwb_sample_t* out) {
    uint8_t raw[6];

    if (!IIS3DWB_DataReady()) return -1;

    _reg_read_multi(IIS3DWB_OUTX_L_A, raw, 6);

    out->x = (int16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    out->y = (int16_t)(raw[2] | ((uint16_t)raw[3] << 8));
    out->z = (int16_t)(raw[4] | ((uint16_t)raw[5] << 8));

        /* Debug: read output registers directly */
    {
        uint8_t r[6];
        _reg_read_multi(IIS3DWB_OUTX_L_A, r, 6);
        extern void uart1_send(const char*);
        char _b[64];
        snprintf(_b,64,"OUT: XL=%02X XH=%02X YL=%02X YH=%02X ZL=%02X ZH=%02X\r\n",
                 r[0],r[1],r[2],r[3],r[4],r[5]);
        uart1_send(_b);
    }
    return 0;
}
