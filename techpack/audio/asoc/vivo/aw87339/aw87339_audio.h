#ifndef __AW87339_H__
#define __AW87339_H__
#if 0
unsigned char aw87339_kspk_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG */
	0x01, 0x06,
	0x02, 0xA3,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x10,
	0x06, 0x07,
	0x07, 0x52,
	0x08, 0x06,
	0x09, 0x08,
	0x0A, 0x96,
	0x62, 0x01,
	0x63, 0x00,
	0x64, 0x00,
	0x01, 0x0E
};

unsigned char aw87339_drcv_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG */
	0x01, 0x02,
	0x02, 0xAB,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x00,
	0x06, 0x0F,
	0x07, 0x52,
	0x08, 0x09,
	0x09, 0x08,
	0x0A, 0x97,
	0x62, 0x01,
	0x63, 0x00,
	0x64, 0x00,
	0x01, 0x0A
};

unsigned char aw87339_abrcv_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG */
	0x01, 0x02,
	0x02, 0xAF,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x00,
	0x06, 0x0F,
	0x07, 0x52,
	0x08, 0x09,
	0x09, 0x08,
	0x0A, 0x97,
	0x62, 0x01,
	0x63, 0x00,
	0x64, 0x00,
	0x01, 0x0A
};

unsigned char aw87339_rcvspk_cfg_default[] = {
	/*0x00, 0x39,  CHIPID REG */
	0x01, 0x06,
	0x02, 0xB3,
	0x03, 0x06,
	0x04, 0x05,
	0x05, 0x00,
	0x06, 0x07,
	0x07, 0x52,
	0x08, 0x06,
	0x09, 0x08,
	0x0A, 0x96,
	0x62, 0x01,
	0x63, 0x00,
	0x64, 0x00,
	0x01, 0x0E
};
#endif

/******************************************************
 *
 *Load config function
 *This driver will use load firmware if AW20036_BIN_CONFIG be defined
 *****************************************************/
#define AWINIC_CFG_UPDATE_DELAY

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define aw87339_REG_CHIPID      0x00
#define aw87339_REG_SYSCTRL     0x01
#define aw87339_REG_MODECTRL    0x02
#define aw87339_REG_CPOVP       0x03
#define aw87339_REG_CPP         0x04
#define aw87339_REG_GAIN        0x05
#define aw87339_REG_AGC3_PO     0x06
#define aw87339_REG_AGC3        0x07
#define aw87339_REG_AGC2_PO     0x08
#define aw87339_REG_AGC2        0x09
#define aw87339_REG_AGC1        0x0A

#define aw87339_REG_DFT1        0x62
#define aw87339_REG_DFT2        0x63
#define aw87339_REG_ENCRY       0x64

#define aw87339_CHIP_DISABLE    0x0C

#define AW87339_CHIPID          0x39

#define REG_NONE_ACCESS         0
#define REG_RD_ACCESS           (1 << 0)
#define REG_WR_ACCESS           (1 << 1)
#define aw87339_REG_MAX         0xFF

const unsigned char aw87339_reg_access[aw87339_REG_MAX] = {
	[aw87339_REG_CHIPID] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_SYSCTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_MODECTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_CPOVP] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_CPP] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_GAIN] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_AGC3_PO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_AGC3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_AGC2_PO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_AGC2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_AGC1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[aw87339_REG_DFT1] = REG_RD_ACCESS,
	[aw87339_REG_DFT2] = REG_RD_ACCESS,
	[aw87339_REG_ENCRY] = REG_RD_ACCESS,
};

#define AW87359_REG_CHIPID		0x00
#define AW87359_REG_SYSCTRL		0x01
#define AW87359_REG_MDCRTL		0x02
#define AW87359_REG_CPOVP		0x03
#define AW87359_REG_CPP			0x04
#define AW87359_REG_PAG			0x05
#define AW87359_REG_AGC3PO		0x06
#define AW87359_REG_AGC3PA		0x07
#define AW87359_REG_AGC2PO		0x08
#define AW87359_REG_AGC2PA		0x09
#define AW87359_REG_AGC1PA		0x0A
#define AW87359_REG_DFT_SYSCTRL		0x61
#define AW87359_REG_DFT_MDCTRL		0x62
#define AW87359_REG_DFT_CPOVP2		0x63
#define AW87359_REG_DFT_AGCPA		0x64
#define AW87359_REG_DFT_POFR		0x65
#define AW87359_REG_DFT_OC		0x66
#define AW87359_REG_DFT_OTA		0x67
#define AW87359_REG_DFT_REF		0x68
#define AW87359_REG_DFT_LDO		0x69
#define AW87359_REG_ENCR		0x70

#define AW87359_CHIPID          0x59

/********************************************
 * Register Access
 *******************************************/
#define AW87359_REG_NONE_ACCESS		(0)
#define AW87359_REG_RD_ACCESS		(1 << 0)
#define AW87359_REG_WR_ACCESS		(1 << 1)
#define AW87359_REG_MAX			(0xFF)

const unsigned char aw87359_reg_access[AW87359_REG_MAX] = {
	[AW87359_REG_CHIPID]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_SYSCTRL]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_MDCRTL]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_CPOVP]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_CPP]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_PAG]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_AGC3PO]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_AGC3PA]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_AGC2PO]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_AGC2PA]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_AGC1PA]	= AW87359_REG_RD_ACCESS | AW87359_REG_WR_ACCESS,
	[AW87359_REG_DFT_SYSCTRL] = AW87359_REG_RD_ACCESS,
	[AW87359_REG_DFT_MDCTRL] = AW87359_REG_RD_ACCESS,
	[AW87359_REG_DFT_CPOVP2] = AW87359_REG_RD_ACCESS,
	[AW87359_REG_DFT_AGCPA]	= AW87359_REG_RD_ACCESS,
	[AW87359_REG_DFT_POFR]	= AW87359_REG_RD_ACCESS,
	[AW87359_REG_DFT_OC]	= AW87359_REG_RD_ACCESS,
	[AW87359_REG_DFT_OTA]	= AW87359_REG_RD_ACCESS,
	[AW87359_REG_DFT_REF]	= AW87359_REG_RD_ACCESS,
	[AW87359_REG_DFT_LDO]	= AW87359_REG_RD_ACCESS,
	[AW87359_REG_ENCR]	= AW87359_REG_RD_ACCESS,
};

struct aw87339_container {
	int len;
	unsigned char data[];
};

struct aw87339 {
	struct i2c_client *i2c_client;
	struct kobject *k_obj;
	int chipid;
	int reset_gpio;
	unsigned char init_flag;
	unsigned char hwen_flag;
	unsigned char kspk_cfg_update_flag;
	unsigned char drcv_cfg_update_flag;
	unsigned char abrcv_cfg_update_flag;
	unsigned char rcvspk_cfg_update_flag;
	const unsigned char *aw873xx_reg_access;
	struct hrtimer cfg_timer;
	struct mutex cfg_lock;
	struct work_struct cfg_work;
	struct delayed_work ram_work;
};

/****************************************************
 * aw87339 functions
 ****************************************************/
unsigned char aw87339_audio_off(void);
unsigned char aw87339_audio_kspk(void);
unsigned char aw87339_audio_drcv(void);
unsigned char aw87339_audio_abrcv(void);
unsigned char aw87339_audio_rcvspk(void);

#endif
