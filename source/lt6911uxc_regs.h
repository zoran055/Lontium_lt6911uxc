#ifndef __LT6911UXE_REGS_H__
#define __LT6911UXE_REGS_H__

/* Control */
#define SW_BANK 	0xff
#define ENABLE_I2C 	0xEE
#define MIPI_TX_CTRL	0xB0

/* Interrupts */
#define INT_VIDEO		0x84
#define INT_VIDEO_DISAPPEAR	0x00
#define INT_VIDEO_READY		0x01

#define INT_AUDIO		0x84
#define INT_AUDIO_DISAPPEAR	0x02
#define INT_AUDIO_READY	0x03

/* Timings */
#define half_PixelClock2 0x85
#define half_PixelClock1 0x86
#define half_PixelClock0 0x87
#define half_Htotal1 0x88
#define half_Htotal0 0x89
#define Vtotal1 0x8A
#define Vtotal0 0x8B
#define half_Hactive1 0x8C
#define half_Hactive0 0x8D
#define Vactive1 0x8E
#define Vactive0 0x8F
#define Audio_FS_Value1  0x90
#define Audio_FS_Value0  0x91
#define ByteClock2 0x92
#define ByteClock1 0x93
#define ByteClock0 0x94
#define MIPI_LANES 0x95
#define MIPI_FORMAT 0x96


#endif  /* __LT6911UXE_REGS_H__ */
