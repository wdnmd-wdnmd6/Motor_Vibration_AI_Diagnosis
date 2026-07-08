/*
 * sensor_dsp.c - IIS3DWB FIFO batch acquisition
 * No timer. No continuous SPI. FIFO collects, main loop reads bursts.
 */

#include "sensor_dsp.h"
#include "hardware.h"
#include "iis3dwb.h"
#include "ch32h417.h"
#include <math.h>
#include <string.h>

volatile uint32_t  dsp_sample_idx   = 0;
volatile uint8_t   dsp_buffer_ready = 0;
accel_sample_t     dsp_buffer[SAMPLE_COUNT];

/* FIFO register addresses (not in iis3dwb.h) */
#define IIS3DWB_FIFO_CTRL1    0x07
#define IIS3DWB_FIFO_CTRL2    0x08
#define IIS3DWB_FIFO_CTRL3    0x09
#define IIS3DWB_FIFO_CTRL4    0x0A
#define IIS3DWB_FIFO_STATUS1  0x3A
#define IIS3DWB_FIFO_STATUS2  0x3B
#define IIS3DWB_FIFO_DATA_TAG 0x78

/* Low-level register access (direct SPI, bypasses iis3dwb.c helpers) */
static uint8_t _r8(uint8_t addr) {
    uint8_t v;
    IIS3DWB_CS_Low();
    SPI2_ReadWriteByte(addr | 0x80);
    v = SPI2_ReadWriteByte(0x00);
    IIS3DWB_CS_High();
    return v;
}
static void _w8(uint8_t addr, uint8_t d) {
    IIS3DWB_CS_Low();
    SPI2_ReadWriteByte(addr & 0x7F);
    SPI2_ReadWriteByte(d);
    IIS3DWB_CS_High();
}

void SensorDSP_Init(void) {
    /* FIFO_CTRL3: BDR_XL = 6.66 kHz (match sensor ODR) */
    _w8(IIS3DWB_FIFO_CTRL3, 0x0A); /* BDR_XL = 6.66kHz */
    /* FIFO_CTRL4: continuous mode */
    _w8(IIS3DWB_FIFO_CTRL4, 0x06);
}

void SensorDSP_Start(void) {
    dsp_sample_idx   = 0;
    dsp_buffer_ready = 0;
}

void SensorDSP_Stop(void) {
}

/* Read all available FIFO samples in one burst. Returns number read. */
static int _fifo_burst(accel_sample_t* dst, int max_n) {
    uint16_t avail;
    {
        uint8_t s1 = _r8(IIS3DWB_FIFO_STATUS1);
        uint8_t s2 = _r8(IIS3DWB_FIFO_STATUS2);
        avail = (uint16_t)s1 | ((uint16_t)(s2 & 0x03) << 8);
    }
    if (avail == 0) return 0;
    if (avail > max_n) avail = max_n;

    IIS3DWB_CS_Low();
    SPI2_ReadWriteByte(IIS3DWB_FIFO_DATA_TAG | 0x80 | 0x40); /* read + auto-inc */
    for (uint16_t i = 0; i < avail; i++) {
        uint8_t tag = SPI2_ReadWriteByte(0x00); (void)tag;
        uint8_t xl = SPI2_ReadWriteByte(0x00);
        uint8_t xh = SPI2_ReadWriteByte(0x00);
        uint8_t yl = SPI2_ReadWriteByte(0x00);
        uint8_t yh = SPI2_ReadWriteByte(0x00);
        uint8_t zl = SPI2_ReadWriteByte(0x00);
        uint8_t zh = SPI2_ReadWriteByte(0x00);
        dst->x = (int16_t)(xl | ((uint16_t)xh << 8));
        dst->y = (int16_t)(yl | ((uint16_t)yh << 8));
        dst->z = (int16_t)(zl | ((uint16_t)zh << 8));
        dst++;
    }
    IIS3DWB_CS_High();
    return (int)avail;
}

int SensorDSP_Service(void) {
    if (dsp_buffer_ready) return 1;

    int space = SAMPLE_COUNT - dsp_sample_idx;
    if (space <= 0) {
        dsp_buffer_ready = 1;
        return 1;
    }

    int n = _fifo_burst(&dsp_buffer[dsp_sample_idx], space);
    if (n > 0) {
        dsp_sample_idx += n;
    }

    if (dsp_sample_idx >= SAMPLE_COUNT) {
        dsp_buffer_ready = 1;
        return 1;
    }
    return 0;
}

/* FFT-based spectral feature extraction (same as before) */
static void fft16(const int16_t* in, float* mag) {
    float re[16], im[16] = {0};
    for (int i = 0; i < 16; i++) re[i] = (float)in[i];
    const uint8_t rev[] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};
    float tmp[16]; for (int i = 0; i < 16; i++) tmp[i] = re[rev[i]];
    memcpy(re, tmp, sizeof(re));
    for (int s = 1; s <= 4; s++) {
        int m = 1 << s;
        float wm_re = cosf(6.2831853f / m), wm_im = -sinf(6.2831853f / m);
        for (int k = 0; k < 16; k += m) {
            float w_re = 1.0f, w_im = 0.0f;
            for (int j = 0; j < m/2; j++) {
                int a = k + j, b = k + j + m/2;
                float u_re = re[a], u_im = im[a];
                float v_re = w_re*re[b] - w_im*im[b];
                float v_im = w_re*im[b] + w_im*re[b];
                re[a] = u_re+v_re; im[a] = u_im+v_im;
                re[b] = u_re-v_re; im[b] = u_im-v_im;
                float wr2 = w_re*wm_re - w_im*wm_im;
                float wi2 = w_re*wm_im + w_im*wm_re;
                w_re = wr2; w_im = wi2;
    }}}
    for (int i = 0; i < 8; i++) mag[i] = sqrtf(re[i]*re[i] + im[i]*im[i]);
}

int SensorDSP_ExtractFeatures(const accel_sample_t* buf, uint32_t count, int8_t* out) {
    if (count < 16) return -1;
    static float combined[12000];
    for (uint32_t i = 0; i < count; i++) {
        float x = buf[i].x, y = buf[i].y, z = buf[i].z;
        combined[i] = sqrtf(x*x + y*y + z*z);
    }
    #define N_FFT 16
    #define N_STEP 8
    float bin_sum[8]={0}; float peak_mag[3]={0}; int peak_bin[3]={0};
    uint32_t n_win = 0;
    for (uint32_t off = 0; off + N_FFT <= count; off += N_STEP) {
        int16_t w[16];
        float _mean = 0; for (int i=0;i<16;i++) _mean += combined[off+i]; _mean /= 16.0f;
        for (int i = 0; i < 16; i++) {
            float v = (combined[off+i] - _mean) / 16.0f;
            if(v>32767)v=32767; if(v<-32768)v=-32768;
            w[i]=(int16_t)v;
        }
        float mag[8]; fft16(w, mag);
        for (int i=0;i<8;i++) {
            bin_sum[i]+=mag[i];
            if(i>0 && mag[i]>peak_mag[0]){peak_mag[2]=peak_mag[1];peak_bin[2]=peak_bin[1];peak_mag[1]=peak_mag[0];peak_bin[1]=peak_bin[0];peak_mag[0]=mag[i];peak_bin[0]=i;}
            else if(i>0 && mag[i]>peak_mag[1]){peak_mag[2]=peak_mag[1];peak_bin[2]=peak_bin[1];peak_mag[1]=mag[i];peak_bin[1]=i;}
            else if(i>0 && mag[i]>peak_mag[2]){peak_mag[2]=mag[i];peak_bin[2]=i;}
        }
        n_win++;
    }
    for (int i=0;i<8;i++) bin_sum[i]/=(float)n_win;
    float band[5];
    band[0]=bin_sum[0]+bin_sum[1]*0.6f;
    band[1]=bin_sum[1]*0.4f+bin_sum[2]+bin_sum[3]+bin_sum[4]+bin_sum[5]+bin_sum[6]+bin_sum[7]*0.5f;
    band[2]=bin_sum[5]*0.5f+bin_sum[6]+bin_sum[7]*0.5f;
    band[3]=band[1]*0.5f; band[4]=band[2]*0.3f;
    float rf[13];
    for(int i=0;i<3;i++){rf[i]=(float)peak_bin[i]/7.0f; rf[3+i]=logf(peak_mag[i]+1.0f);}
    for(int i=0;i<5;i++) rf[6+i]=logf(band[i]+1.0f);
    float am=0; for(int i=0;i<8;i++) am+=bin_sum[i]; am/=8;
    rf[11]=logf(am+1.0f); rf[12]=(float)n_win/1500.0f;
    for(int i=0;i<13;i++){
        float v=rf[i]*10.0f;
        if(v>127)v=127; if(v<-128)v=-128;
        out[i]=(int8_t)v;
    }
    return 0;
}