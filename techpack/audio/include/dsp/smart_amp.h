#ifndef _SMART_AMP_H
#define _SMART_AMP_H

#include <linux/types.h>
#include <dsp/apr_audio-v2.h>
#include <linux/delay.h>

#ifdef VIVO_PORT_SMARTPA
#define CAPI_V2_VIVO_SP_RX                     0x100D1003
#define CAPI_V2_VIVO_SP_TX                     0x100D1004
#define VIVO_SP_WORK_MODE_RX                   0x110D0001
#define VIVO_SP_WORK_MODE_TX                   0x110D0006
#define VIVO_SP_CALIBRATION                    0x110D0008
#define VIVO_SP_WORK_MODE_RX2                  VIVO_SP_WORK_MODE_RX+(1<<8)
#define VIVO_SP_WORK_MODE_TX2                  VIVO_SP_WORK_MODE_TX+(1<<8)
#define VIVO_SP_CALIBRATION2                   VIVO_SP_CALIBRATION+(1<<8)
struct vivo_calibration_rx
{
	uint32_t re_q15;
	uint32_t f0_q15;
	uint32_t qts_q15;
	uint32_t calibration_success;

	uint32_t pa_id;//hex format
	uint32_t v_max_q15;//V sense full scale
	uint32_t i_max_q15;//I sense full scale
	uint32_t v_out_max_q15;//output max voltage
};
/* user impedance vivo: (detect_val >> 15)*/
#define VIVO_TRANSF_IMPED_TO_USER_I(X) \
		((X * 100) >> 15) / 100
#define VIVO_TRANSF_IMPED_TO_USER_M(X) \
		((X * 100) >> 15) % 100
#endif

/* Below 3 should be same as in aDSP code */
#define AFE_PARAM_ID_SMARTAMP_DEFAULT   0x10001166
#define AFE_SMARTAMP_MODULE             0x11111112  /*Rx module*/
#define AFE_SMARTAMP_MODULE_TX          0x11111111  /*Tx module*/
#define SMART_AMP
#define MAX_DSP_PARAM_INDEX		600

#define TAS_GET_PARAM		1
#define TAS_SET_PARAM		0
#define TAS_PAYLOAD_SIZE	14
#define TAS_RX_PORT_ID		0x1004 /* TERT MI2S RX */
#define TAS_TX_PORT_ID		0x1005 /* TERT MI2S TX */
#define SLAVE1		0x98
#define SLAVE2		0x9A
#define SLAVE3		0x9C
#define SLAVE4		0x9E
#define SMARTAMP_STATUS_NORMAL 0
#define SMARTAMP_STATUS_BYPASS 1
#define SMARTAMP_STATUS_MUTE   2

#define AFE_SA_GET_F0          3810
#define AFE_SA_GET_Q           3811
#define AFE_SA_GET_TV          3812
#define AFE_SA_GET_RE          3813
#define AFE_SA_CALIB_INIT      3814
#define AFE_SA_CALIB_DEINIT    3815
#define AFE_SA_SET_RE          3816
#define AFE_SA_F0_TEST_INIT    3817
#define AFE_SA_F0_TEST_DEINIT  3818
#define AFE_SA_SET_PROFILE     3819
#define AFE_SA_SET_STATUS      3820

#define CAPI_V2_TAS_TX_ENABLE 0x00012D14
#define CAPI_V2_TAS_TX_CFG    0x00012D16

#define CAPI_V2_TAS_RX_ENABLE 0x00012D13
#define CAPI_V2_TAS_RX_CFG    0x00012D15
#define AFE_SA_IS_SPL_IDX(X)	((((X) >= 3810) && ((X) < 3899)) ? 1 : 0)
#if 0 //static integration values
#define AFE_SA_CALIB_CTRL_START	3814
#define AFE_SA_CALIB_CTRL_STOP	3815
#define AFE_SA_SET_PROFILE		3817
#define AFE_SA_SET_RE			3818
#define AFE_SA_SET_STATUS		3819
#endif

struct afe_smartamp_set_params_t {
	uint32_t payload[TAS_PAYLOAD_SIZE];
} __packed;

struct afe_smartamp_get_params_t {
    uint32_t payload[TAS_PAYLOAD_SIZE];
} __packed;

struct afe_smartamp_calib_get_resp {
	uint32_t status;
	struct param_hdr_v3 pdata;
	struct afe_smartamp_get_params_t param;
} __packed;

int afe_smartamp_get_calib_data(struct afe_smartamp_calib_get_resp *calib_resp,
		uint32_t param_id, uint32_t module_id);

int afe_smartamp_set_calib_data(uint32_t param_id,struct afe_smartamp_set_params_t *prot_config,
		uint8_t length, uint32_t module_id);

int afe_smartamp_algo_ctrl(u8 *data, uint32_t param_id, uint8_t dir,
		int32_t size, int module_id);

int smartamp_set_status(uint32_t st);

void smartpa_set_mi2s_port(int port_rx, int port_tx);

#endif
