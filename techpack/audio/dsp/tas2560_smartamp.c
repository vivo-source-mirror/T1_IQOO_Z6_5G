/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
** File:
**     tas2560_smartamp.c
**
** Description:
**     TAS2560 Set/Get data for tas2560 smartamp algorithm.
**
** =============================================================================
*/
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <dsp/msm_audio_ion.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/audio_cal_utils.h>
#include <dsp/q6afe-v2.h>
#include <dsp/q6audio-v2.h>
#include <ipc/apr_tal.h>
#include "adsp_err.h"
#include <dsp/smart_amp.h>

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug pr_info

#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg dev_info

#ifdef SMART_AMP
static uint32_t s_smartamp_status = SMARTAMP_STATUS_BYPASS;
static struct mutex routing_lock;
#endif

int afe_smartamp_algo_ctrl(u8 *user_data, uint32_t param_id,
			uint8_t get_set, int32_t length, int module_id)
{
	int32_t  ret = 0;

	switch (get_set) {
	case TAS_SET_PARAM:
			ret = afe_smartamp_set_calib_data(param_id ,
					(struct afe_smartamp_set_params_t *)user_data, length, module_id);
		break;

	case TAS_GET_PARAM: {
			struct afe_smartamp_calib_get_resp calib_resp;
			memset (&calib_resp, 0, sizeof(calib_resp));

			ret = afe_smartamp_get_calib_data(&calib_resp,
							param_id, module_id);

			memcpy(user_data, calib_resp.param.payload, length);
		}
		break;
	default:
		goto fail_cmd;
	}

fail_cmd:
	return ret;
}
EXPORT_SYMBOL(afe_smartamp_algo_ctrl);

int32_t adm_smartamp_set_re(u8 *re, uint8_t sz)
{
	int param_id = AFE_SA_SET_RE;
	struct afe_smartamp_set_params_t prot_config;
	param_id |= (1<<24) | (sz << 16);

	memcpy (prot_config.payload, (void*)re, sz);
	return afe_smartamp_set_calib_data(param_id, &prot_config, sz
				, AFE_SMARTAMP_MODULE);
}
EXPORT_SYMBOL(adm_smartamp_set_re);

int32_t adm_smartamp_set_status(u8 *status, uint8_t sz)
{
	int param_id = AFE_SA_SET_STATUS;
	struct afe_smartamp_set_params_t prot_config;
	param_id |= (1<<24) | (sz << 16);

	pr_err("%s: status: %d\n", __func__, *status);
	memcpy (prot_config.payload, (void*)status, sz);
	return afe_smartamp_set_calib_data(param_id, &prot_config, sz
			, AFE_SMARTAMP_MODULE);
}
EXPORT_SYMBOL(adm_smartamp_set_status);

int smartamp_set_status(uint32_t st)
{
	int32_t ret = 0;
	uint32_t status = st;
	pr_info ("[smartamp] setting st=0x%x", st);

	mutex_lock(&routing_lock);
	s_smartamp_status = status;
	ret = adm_smartamp_set_status ((uint8_t*)&status, sizeof(int32_t));
	mutex_unlock(&routing_lock);
	return 0;
}
EXPORT_SYMBOL(smartamp_set_status);
