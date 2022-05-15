#ifndef __AWINIC_DSP_H__
#define __AWINIC_DSP_H__



/*#define AW_MTK_PLATFORM*/
#define AW_QCOM_PLATFORM

#define AWINIC_DSP_MSG_HDR_VER (1)
#define AW_DSP_TRY_TIME (3)
#define AW_DSP_SLEEP_TIME (10)
#define AFE_PORT_ID_AWDSP_RX (0x1002)     /*AFE_PORT_ID_SEC_MI2S_RX*/
#define AW_COPP_MODULE_ID (0X10013D02)			/*SKT module id*/
#define AW_COPP_MODULE_PARAMS_ID_EN (0X10013D14)	/*SKT enable param id*/

#define INLINE_PARAM_ID_NULL				(0x00000000)
#define INLINE_PARAM_ID_ACTIVE_FLAG			(0x00000004)
#define INLINE_PARAM_ID_REAL_DATA			(0x00000005)
#define INLINE_PARAM_ID_DIRECT_CURRENT_FLAG		(0x00000006)
#define INLINE_PARAMS_ID_SPK_STATUS			(0x00000007)
#define INLINE_PARAMS_ID_VERSION			(0x00000008)


int aw_write_data_to_dsp(int index, void *data, int data_size, int channel);
int aw_read_data_from_dsp(int index, void *data, int data_size, int channel);
int aw_write_msg_to_dsp(int inline_id, void *data, int data_size, int channel);
int aw_read_msg_from_dsp(int inline_id, void *data, int data_size, int channel);
int aw_send_module_enable(void *buf, uint8_t type);
int aw_get_module_enable(void *buf, uint8_t type);
int aw_get_f0_q(struct f0_q_data *data, int data_size, int channel);
int aw_get_algo_version(unsigned int *data);
int aw_dsp_copp_module_en(bool enable);

#endif
