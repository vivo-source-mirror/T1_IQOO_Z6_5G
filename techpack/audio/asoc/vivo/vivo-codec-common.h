/* Add by ChenJinQuan<chenjinquan@vivo.com> for vivo codec. */
#ifndef __VIVO_CODEC_COMMON_H__
#define __VIVO_CODEC_COMMON_H__

#define INVAL_GPIO -1

enum {
	I2S_IN = 0,
	I2S_OUT,
};

enum {
	I2S_RATE_44_1_KHZ = 0,
	I2S_RATE_48_KHZ,
	I2S_RATE_88_2_KHZ,
	I2S_RATE_96_KHZ,
	I2S_RATE_176_4_KHZ,
	I2S_RATE_192_KHZ,
};

enum {
	I2S_FORMAT_S16_LE = 0,
	I2S_FORMAT_S24_LE,
};

enum {
	HP_PATH_OFF = 0,
	HP_PATH_NORMAL,
	HP_PATH_HIFI,
	HP_PATH_HIFI_BYPASS_ON,
	HP_PATH_HIFI_BYPASS_OFF,
};

enum {
	MIC_PATH_OFF = 0,
	MIC_PATH_NORMAL,
	MIC_PATH_KTV,
};

enum {
	SMART_PA_MODE_OFF = 0,
	SMART_PA_MODE_MUSIC,
	SMART_PA_MODE_VOICE,
	SMART_PA_MODE_KTV,
};

enum {
	OFF = 0,
	ON,
};

/*
 * External codec id, defined for power up/down,
 * hw reset and gpio setting.
 */
enum vivo_codec_id {
	VIVO_CODEC_SMART_PA = 0,
	VIVO_CODEC_HIFI_DAC,
	VIVO_CODEC_HIFI_CLK,
	VIVO_CODEC_KTV,
	VIVO_CODEC_HIFI_DAC_RST_DOWN,
};

enum reset_mode_id {
	VIVO_CODEC_RST_PU = 0,
	VIVO_CODEC_RST_PD,
};

enum hs_swch_state {
	VIVO_CODEC_SWCH_OFF = 0,
	VIVO_CODEC_SWCH_ON_NORMAL,
	VIVO_CODEC_SWCH_ON_HIFI,
};

enum ext_pa_swich_state {
	EXT_PA_SWICH_NONE = 0,
	EXT_PA_SWICH_SPK,
	EXT_PA_SWICH_RCV,
	EXT_PA_SWICH_SPK_FM,
};

enum cs_bypass_swich_state {
	CS_BYPASS_SWICH_NONE = 0,
	CS_BYPASS_SWICH_DEFAULT,
	CS_BYPASS_SWICH_ON,
};

struct audio_params
{
	int rate;
	unsigned int i2s_format;
	int pcm_format;
	unsigned long sys_clk;
	/* Each external codec has their private params */
	void *private_params;
};

struct vivo_codec_function
{
	/*
	 * Called by external codecs driver,
	 * such as smart pa, hifi dac, ktv codec etc.
	 */
	int (*power_up)(enum vivo_codec_id id);
	int (*power_down)(enum vivo_codec_id id);
	int (*hw_reset)(enum vivo_codec_id id);
	int (*pull_down_reset)(enum vivo_codec_id id);
	int (*mclk_enable)(enum vivo_codec_id id, unsigned long rate);
	int (*mclk_disable)(enum vivo_codec_id id);
	int (*mi2s_clk_enable)(struct audio_params *params);
	int (*mi2s_clk_disable)(void);

	/* Called by vivo codec driver */
	int (*hifi_clk_enable)(struct audio_params *params, bool enable);
	int (*hifi_dac_enable)(struct audio_params *params, bool enable);
	int (*hifi_dac_mute)(int mute);
	int (*smart_pa_enable)(struct audio_params *params, bool enable, int mode);
	int (*smart_pa_set_mode)(int mode);
	int (*smart_pa_mute)(int mute);

	/* for mbhc */
	int (*get_hp_switch_state)(void);
	int (*set_hp_switch_state)(int state);
	int (*get_mic_switch_state)(void);
	int (*set_mic_switch_state)(int state);
	int (*get_hifi_dac_state)(void);
	void (*smartpa_debug_retry_probe)(void);
	unsigned int (*mbhc_get_hp_impedance)(void);

	/* for ext pa */
	int (*ext_pa_enable)(int state);
	bool (*ext_get_pa_state)(void);
	/* for cs bypass */
	int (*cs_bypass_enable)(int state);
	int (*msm_mi2s_set_mclk)(bool enable);
};

struct vivo_codec_function * get_vivo_codec_function(void);
void set_vivo_codec_function(struct vivo_codec_function *fun);
int get_pa_value(void);
void set_pa_value(int value);
void get_smartpa_lock(void);
void release_smartpa_lock(void);
#endif //__VIVO_CODEC_COMMON_H__
