#ifndef USB_CLOCK_H
#define USB_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 仅启用主PLL的 48MHz（PLLQ）给 USB OTG FS 使用，不改变系统主频（仍 HSI 16MHz） */
void USB_EnablePLL48CLK(void);

#ifdef __cplusplus
}
#endif

#endif 
