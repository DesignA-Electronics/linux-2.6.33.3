/*
 * Texas Instruments TLV320AIC26 low power audio CODEC
 * register definitions
 *
 * Copyright (C) 2008 Secret Lab Technologies Ltd.
 */

#ifndef _TLV320AIC16_H_
#define _TLV320AIC16_H_

/* AIC26 Registers */
#define AIC26_READ_COMMAND_WORD(addr)	((1 << 15) | (addr << 5))
#define AIC26_WRITE_COMMAND_WORD(addr)	((0 << 15) | (addr << 5))
#define AIC26_PAGE_ADDR(page, offset)	((page << 6) | offset)
#define AIC26_NUM_REGS			AIC26_PAGE_ADDR(3, 0)

/* Page 0: Auxillary data registers */
#define AIC26_REG_BAT1			AIC26_PAGE_ADDR(0, 0x05)
#define AIC26_REG_BAT2			AIC26_PAGE_ADDR(0, 0x06)
#define AIC26_REG_AUX			AIC26_PAGE_ADDR(0, 0x07)
#define AIC26_REG_TEMP1			AIC26_PAGE_ADDR(0, 0x09)
#define AIC26_REG_TEMP2			AIC26_PAGE_ADDR(0, 0x0A)

/* Page 1: Auxillary control registers */
#define AIC26_REG_AUX_ADC		AIC26_PAGE_ADDR(1, 0x00)
#define AIC26_REG_STATUS		AIC26_PAGE_ADDR(1, 0x01)
#define AIC26_REG_REFERENCE		AIC26_PAGE_ADDR(1, 0x03)
#define AIC26_REG_RESET			AIC26_PAGE_ADDR(1, 0x04)

/* Page 2: Audio control registers */
#define AIC26_REG_AUDIO_CTRL1		AIC26_PAGE_ADDR(2, 0x00)
#define AIC26_REG_ADC_GAIN		AIC26_PAGE_ADDR(2, 0x01)
#define AIC26_REG_DAC_GAIN		AIC26_PAGE_ADDR(2, 0x02)
#define AIC26_REG_SIDETONE		AIC26_PAGE_ADDR(2, 0x03)
#define AIC26_REG_AUDIO_CTRL2		AIC26_PAGE_ADDR(2, 0x04)
#define AIC26_REG_POWER_CTRL		AIC26_PAGE_ADDR(2, 0x05)
#define AIC26_REG_AUDIO_CTRL3		AIC26_PAGE_ADDR(2, 0x06)

#define AIC26_REG_FILTER_COEFF_L_N0	AIC26_PAGE_ADDR(2, 0x07)
#define AIC26_REG_FILTER_COEFF_L_N1	AIC26_PAGE_ADDR(2, 0x08)
#define AIC26_REG_FILTER_COEFF_L_N2	AIC26_PAGE_ADDR(2, 0x09)
#define AIC26_REG_FILTER_COEFF_L_N3	AIC26_PAGE_ADDR(2, 0x0A)
#define AIC26_REG_FILTER_COEFF_L_N4	AIC26_PAGE_ADDR(2, 0x0B)
#define AIC26_REG_FILTER_COEFF_L_N5	AIC26_PAGE_ADDR(2, 0x0C)
#define AIC26_REG_FILTER_COEFF_L_D1	AIC26_PAGE_ADDR(2, 0x0D)
#define AIC26_REG_FILTER_COEFF_L_D2	AIC26_PAGE_ADDR(2, 0x0E)
#define AIC26_REG_FILTER_COEFF_L_D4	AIC26_PAGE_ADDR(2, 0x0F)
#define AIC26_REG_FILTER_COEFF_L_D5	AIC26_PAGE_ADDR(2, 0x10)
#define AIC26_REG_FILTER_COEFF_R_N0	AIC26_PAGE_ADDR(2, 0x11)
#define AIC26_REG_FILTER_COEFF_R_N1	AIC26_PAGE_ADDR(2, 0x12)
#define AIC26_REG_FILTER_COEFF_R_N2	AIC26_PAGE_ADDR(2, 0x13)
#define AIC26_REG_FILTER_COEFF_R_N3	AIC26_PAGE_ADDR(2, 0x14)
#define AIC26_REG_FILTER_COEFF_R_N4	AIC26_PAGE_ADDR(2, 0x15)
#define AIC26_REG_FILTER_COEFF_R_N5	AIC26_PAGE_ADDR(2, 0x16)
#define AIC26_REG_FILTER_COEFF_R_D1	AIC26_PAGE_ADDR(2, 0x17)
#define AIC26_REG_FILTER_COEFF_R_D2	AIC26_PAGE_ADDR(2, 0x18)
#define AIC26_REG_FILTER_COEFF_R_D4	AIC26_PAGE_ADDR(2, 0x19)
#define AIC26_REG_FILTER_COEFF_R_D5	AIC26_PAGE_ADDR(2, 0x1A)

#define AIC26_REG_PLL_PROG1		AIC26_PAGE_ADDR(2, 0x1B)
#define AIC26_REG_PLL_PROG2		AIC26_PAGE_ADDR(2, 0x1C)
#define AIC26_REG_AUDIO_CTRL4		AIC26_PAGE_ADDR(2, 0x1D)
#define AIC26_REG_AUDIO_CTRL5		AIC26_PAGE_ADDR(2, 0x1E)

/* fsref dividers; used in register 'Audio Control 1' */
enum aic26_divisors {
	AIC26_DIV_1	= 0,
	AIC26_DIV_1_5	= 1,
	AIC26_DIV_2	= 2,
	AIC26_DIV_3	= 3,
	AIC26_DIV_4	= 4,
	AIC26_DIV_5	= 5,
	AIC26_DIV_5_5	= 6,
	AIC26_DIV_6	= 7,
};

/* Digital data format */
enum aic26_datfm {
	AIC26_DATFM_I2S		= 0 << 8,
	AIC26_DATFM_DSP		= 1 << 8,
	AIC26_DATFM_RIGHTJ	= 2 << 8, /* right justified */
	AIC26_DATFM_LEFTJ	= 3 << 8, /* left justified */
};

/* Sample word length in bits; used in register 'Audio Control 1' */
enum aic26_wlen {
	AIC26_WLEN_16	= 0 << 10,
	AIC26_WLEN_20	= 1 << 10,
	AIC26_WLEN_24	= 2 << 10,
	AIC26_WLEN_32	= 3 << 10,
};

extern struct snd_soc_dai aic26_dai;
extern struct snd_soc_codec_device aic26_soc_codec_dev;

extern void aic26_awds_pwdn(struct snd_soc_codec *codec, int set);

#endif /* _TLV320AIC16_H_ */
