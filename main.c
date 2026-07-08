/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Description        : Motor fault detection V5F - real sensor + inference
 *******************************************************************************/

#include "debug.h"
#include "hardware.h"
#include "lcd.h"
#include "cn_font.h"
#include "inference.h"
#include "iis3dwb.h"
#include "sensor_dsp.h"
#include <stdio.h>
#include <string.h>

static const uint16_t CN_TITLE[]    = { 0x7535,0x673A,0x6545,0x969C,0x68C0,0x6D4B, 0 };
static const uint16_t CN_INIT[]     = { 0x6A21,0x578B,0x521D,0x59CB,0x5316, 0 };
static const uint16_t CN_RESULT[]   = { 0x9884,0x6D4B,0x7ED3,0x679C, 0 };
static const uint16_t CN_CONF[]     = { 0x7F6E,0x4FE1,0x5EA6, 0 };
static const uint16_t CN_NORMAL[]   = { 0x6B63,0x5E38, 0 };
static const uint16_t CN_FAULT[]    = { 0x6545,0x969C, 0 };
static const uint16_t CN_NOSENSOR[] = { 0x4F20,0x611F,0x5668,0x672A,0x63A5,0x5165, 0 };
static const uint16_t* get_label(int idx) { return (idx == 0) ? CN_NORMAL : CN_FAULT; }

static void ftoa(char* b, float f, int d) {
    if (f < 0.0f) { *b++ = '-'; f = -f; }
    int ip = (int)f; char t[12]; int n=0;
    if (ip==0) t[n++]='0'; else while(ip){t[n++]='0'+ip%10;ip/=10;}
    while(n)*b++=t[--n];
    if(d>0){*b++='.'; float fr=f-(float)(int)f;
        for(int i=0;i<d;i++){fr*=10.0f;int di=(int)fr;fr-=di;*b++='0'+di;}}
    *b=0;
}

int main(void)
{
    SystemAndCoreClockUpdate();
    Delay_Init();
    USART1_PA9_Init();
    uart1_send("\r\n===== V5F =====\r\n");

    LCD_Init();
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_DrawCNStr(CN_TITLE, 0, 3, LCD_COLOR_WHITE, LCD_COLOR_BLACK);
    LCD_DrawCNStr(CN_INIT,  0, 24, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);

    if (ei_inference_init() != 0) { uart1_send("Model FAIL\r\n"); while(1); }
    uart1_send("Model OK\r\n");
    IIS3DWB_Init();
    uart1_send("Sensor OK\r\n");
    SensorDSP_Init();
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_DrawCNStr(CN_TITLE, 0, 3, LCD_COLOR_WHITE, LCD_COLOR_BLACK);

    while (1) {
        int8_t features[13]; float scores[3]; char dbg[64];

        SensorDSP_Start();
        while (!dsp_buffer_ready) { SensorDSP_Service(); Delay_Ms(10); }

        /* Stats */
        {
            int16_t mnx=32767,mxx=-32768,mny=32767,mxy=-32768,mnz=32767,mxz=-32768;
            for(uint32_t i=0;i<SAMPLE_COUNT;i++) {
                if(dsp_buffer[i].x<mnx)mnx=dsp_buffer[i].x;if(dsp_buffer[i].x>mxx)mxx=dsp_buffer[i].x;
                if(dsp_buffer[i].y<mny)mny=dsp_buffer[i].y;if(dsp_buffer[i].y>mxy)mxy=dsp_buffer[i].y;
                if(dsp_buffer[i].z<mnz)mnz=dsp_buffer[i].z;if(dsp_buffer[i].z>mxz)mxz=dsp_buffer[i].z;
            }
            {
            int v1=(mnx*100)/2048, v2=(mxx*100)/2048, v3=(mny*100)/2048, v4=(mxy*100)/2048;
            int v5=(mnz*100)/2048, v6=(mxz*100)/2048;
            snprintf(dbg,64,"X:%d.%02d~%d.%02d Y:%d.%02d~%d.%02d Z:%d.%02d~%d.%02dg\r\n",
                v1/100,(v1<0?-v1:v1)%100, v2/100,(v2<0?-v2:v2)%100,
                v3/100,(v3<0?-v3:v3)%100, v4/100,(v4<0?-v4:v4)%100,
                v5/100,(v5<0?-v5:v5)%100, v6/100,(v6<0?-v6:v6)%100);
        }uart1_send(dbg);
        }

        /* Vibration check */
        {
            int16_t mnz=32767,mxz=-32768;
            for(uint32_t i=0;i<SAMPLE_COUNT;i++){
                if(dsp_buffer[i].z<mnz)mnz=dsp_buffer[i].z;if(dsp_buffer[i].z>mxz)mxz=dsp_buffer[i].z;
            }
            if((mxz-mnz)<1000)  /* ~0.5g threshold */{
                uart1_send("No vib\r\n");
                LCD_Clear(LCD_COLOR_BLACK);
                LCD_DrawCNStr(CN_TITLE,0,3,LCD_COLOR_WHITE,LCD_COLOR_BLACK);
                LCD_DrawCNStr(CN_NOSENSOR,0,24,LCD_COLOR_RED,LCD_COLOR_BLACK);
                Delay_Ms(2000); continue;
            }
        }

        SensorDSP_ExtractFeatures(dsp_buffer, SAMPLE_COUNT, features);
        {
            snprintf(dbg,64,"F:%d %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
            features[0],features[1],features[2],features[3],features[4],features[5],features[6],
            features[7],features[8],features[9],features[10],features[11],features[12]);
        uart1_send(dbg);
        }
        for(int i=0;i<13;i++){ int v=features[i]*4; features[i]=(v>127)?127:((v<-128)?-128:v); }
        ei_inference_scores(features, 13, scores);

        int class_idx=0; float conf=scores[0];
        if(scores[1]>conf){class_idx=1;conf=scores[1];}
        if(scores[2]>conf){class_idx=2;conf=scores[2];}
        int is_normal=(class_idx==0);

        LCD_Clear(LCD_COLOR_BLACK);
        LCD_DrawCNStr(CN_TITLE,0,3,LCD_COLOR_WHITE,LCD_COLOR_BLACK);
        LCD_DrawCNStr(CN_RESULT,0,24,LCD_COLOR_WHITE,LCD_COLOR_BLACK);
        LCD_DrawCNStr(get_label(class_idx),0,46,is_normal?LCD_COLOR_GREEN:LCD_COLOR_RED,LCD_COLOR_BLACK);
        LCD_DrawCNStr(CN_CONF,0,68,LCD_COLOR_WHITE,LCD_COLOR_BLACK);
        LCD_DrawFloat16(conf,4,64,68,LCD_COLOR_CYAN,LCD_COLOR_BLACK);

        uart1_send(is_normal?"NORMAL":"FAULT");
        uart1_send(" conf=");ftoa(dbg,conf,4);uart1_send(dbg);uart1_send("\r\n");
        Delay_Ms(2000);
    }
}