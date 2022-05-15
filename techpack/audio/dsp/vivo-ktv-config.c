#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <sound/control.h>
#include <dsp/q6adm-v2.h>
#include <sound/asound.h>
#include <dsp/q6audio-v2.h>
#include <sound/tlv.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/q6common.h>

#include "vivo-ktv-config.h"

#define KTV_PARAMS_SIZE_MAX		64
#define TIMEOUT_MS		1000

struct ktv_ctl {
	atomic_t error_code;
	atomic_t cmd_state;
	/* ktv function state */
	atomic_t ktv_state;
	atomic_t wait_stat;
	int copp_idx;
	int port_id;
	int topology;
	wait_queue_head_t wait;
};

static struct ktv_ctl	this_ktv;
static int ktv_get_params[KTV_PARAMS_SIZE_MAX];

void vivo_ktv_reset(void)
{
	atomic_set(&this_ktv.error_code, 0);
	atomic_set(&this_ktv.cmd_state, 0);
	atomic_set(&this_ktv.ktv_state, 0);
	atomic_set(&this_ktv.wait_stat, 0);
	this_ktv.copp_idx = 0;
	this_ktv.port_id = 0;
	this_ktv.topology = 0;
}

void set_vivo_ktv_state(int state)
{
	atomic_set(&this_ktv.ktv_state, !!state);
}

int get_vivo_ktv_state(void)
{
	return atomic_read(&this_ktv.ktv_state);
}

void set_vivo_ktv_copp_idx(int topology, int copp_idx, int port_id)
{
	if (topology == AUDIO_TX_KTV_TOPOLOGY) {
		this_ktv.copp_idx = copp_idx;
		this_ktv.port_id = port_id;
	}
}

int get_vivo_ktv_topo_state(void)
{
	this_ktv.topology = adm_get_topology_for_port_copp_idx(
		this_ktv.port_id, this_ktv.copp_idx);

	return this_ktv.topology;
}

void set_vivo_ktv_cmd_state(int state)
{
	atomic_set(&this_ktv.cmd_state, !!state);
}

int get_vivo_ktv_cmd_state(void)
{
	return atomic_read(&this_ktv.cmd_state);
}

int get_ktv_apr_err(void)
{
	int apr_err = atomic_read(&this_ktv.error_code);

	if (apr_err) {
		pr_err("%s: adsp return error %d\n",
			__func__, apr_err);
		atomic_set(&this_ktv.error_code, 0);
		return -EINVAL;
	}

	return 0;
}

int ktv_basic_callback(uint32_t *payload, u32 payload_size)
{
	pr_debug("%s: cmd_state = %d\n", __func__,
	         get_vivo_ktv_cmd_state());

	if (get_vivo_ktv_cmd_state() != 1)
		return 0;

	if (payload_size == sizeof(uint32_t))
		atomic_set(&this_ktv.error_code, payload[0]);
	else if (payload_size == (2*sizeof(uint32_t)))
		atomic_set(&this_ktv.error_code, payload[1]);

	set_vivo_ktv_cmd_state(0);
	atomic_set(&this_ktv.wait_stat, 1);
	wake_up(&this_ktv.wait);

	return 1;
}

void ktv_params_callback(uint32_t *payload, u32 payload_size)
{
	int i = 0, length = 0;
	int max_length = sizeof(ktv_get_params) / sizeof(ktv_get_params[0]);

	if (payload[0] != 0)
		pr_err("%s: adsp returned error = 0x%x\n",
			__func__, payload[0]);

	pr_debug("%s: payload_size %d\n", __func__, payload_size);

	/* payload[1]/4 - 2 */
	length =
		(payload[1] - sizeof(struct adm_ktv_common_param)) / sizeof(uint32_t);
	for (i = 0; i < length; i++) {
		pr_info("%s: payload[%d] = %d\n", __func__,
			i + 3, payload[i+3]);
	}

	if (payload_size > (3 * sizeof(uint32_t))) {
		int params_base =
				sizeof(struct adm_ktv_common_param) / sizeof(uint32_t);/* 2 */

		if (length > (max_length - 1))
			length = max_length - 1;

		ktv_get_params[0] = length;
		pr_debug("get ktv param: received parameter length: %x (sizeof(uint32_t))\n",
				ktv_get_params[0]);

		/* storing param size then params */
		for (i = 0; i < length; i++)
			ktv_get_params[1+i] = payload[params_base+1+i];
	} else {
	    ktv_get_params[0] = 0;
	}

	atomic_set(&this_ktv.wait_stat, 1);
	wake_up(&this_ktv.wait);
}

/* Re-achieve adm send params due to interface is changed by fanyongxiang. */
static int vivo_adm_get_params(int port_id, int copp_idx, uint32_t module_id,
		   uint32_t param_id, uint32_t params_length, char *param_value)
{
	struct param_hdr_v3 param_hdr;
	int ret = 0;

	if (!param_value)
		return -ENOMEM;

	memset(&param_hdr, 0, sizeof(param_hdr));
	param_hdr.module_id = module_id;
	param_hdr.instance_id = INSTANCE_ID_TX;
	param_hdr.param_id = param_id;
	param_hdr.param_size = params_length;

	ret = adm_get_pp_params(port_id, copp_idx,
		ADM_CLIENT_ID_DEFAULT, NULL, &param_hdr, (u8 *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed: param_id %d, ret %d.\n",
			__func__, param_id, ret);
		ret = -EINVAL;
	}

	return ret;
}

static int vivo_adm_send_params(u8 *params, u32 param_id, u16 param_size)
{
	u32 packed_param_size = (sizeof(struct param_hdr_v3) + param_size);
	struct param_hdr_v3 param_hdr;
	u8 *packed_params = NULL;
	int ret = 0;

	packed_params = kzalloc(packed_param_size, GFP_KERNEL);
	if (!packed_params)
		return 0;

	memset(&param_hdr, 0, sizeof(param_hdr));
	param_hdr.module_id = CAPI_V2_MODULE_KTV;
	param_hdr.instance_id = INSTANCE_ID_TX;
	param_hdr.param_id = param_id;
	param_hdr.param_size = param_size;

	packed_param_size = 0;
	ret = q6common_pack_pp_params(packed_params,
				&param_hdr, (u8 *)params,
				&packed_param_size);
	if (ret) {
		pr_err("%s: Failed to pack params, error %d\n",
			   __func__, ret);
		goto done;
	}

	ret = adm_set_pp_params(this_ktv.port_id,
				 this_ktv.copp_idx, NULL,
				 packed_params,
				 packed_param_size);
	if (ret) {
		pr_err("%s: Setting param failed with err=%d\n",
			__func__, ret);
		ret = -EINVAL;
	}

done:
	kfree(packed_params);
	packed_params = NULL;
	return ret;
}

static uint32_t ktv_hp_rx_port;
static uint32_t ktv_spk_rx_port;
void vivo_audio_ktv_get_rx_port_id(uint32_t hp_port_id,
		uint32_t spk_port_id)
{
	ktv_hp_rx_port = hp_port_id;
	ktv_spk_rx_port = spk_port_id;
}
EXPORT_SYMBOL(vivo_audio_ktv_get_rx_port_id);

int vivo_audio_ktv_open(uint32_t flag)
{
	int ret = 0;
	struct adm_cmd_open_vivo_audio_ktv open;

	open.enable_flags = flag;
	open.hp_rx_port_id = ktv_hp_rx_port;
	open.spk_rx_port_id = ktv_spk_rx_port;

	pr_info("%s: enable_flag: %d, ktv_hp_rx_port: 0x%x,"
		"ktv_spk_rx_port: 0x%x,\n", __func__, flag,
		ktv_hp_rx_port, ktv_spk_rx_port);

	open.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	open.hdr.pkt_size = sizeof(open);
	open.hdr.src_svc = APR_SVC_ADM;
	open.hdr.src_domain = APR_DOMAIN_APPS;
	open.hdr.src_port = 0;
	open.hdr.dest_svc = APR_SVC_ADM;
	open.hdr.dest_domain = APR_DOMAIN_ADSP;
	open.hdr.dest_port = 0;
	open.hdr.token = 0;
	open.hdr.opcode = ADM_CMD_KTV_OPEN;

	atomic_set(&this_ktv.wait_stat, 0);
	ret = vivo_send_adm_apr_pkt((uint32_t *)&open);
	if (ret < 0) {
		pr_err("%s open vivo audio ktv fail!!\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_ktv.wait,
                         atomic_read(&this_ktv.wait_stat),
                         msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s wait for event timeout!!!\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = get_ktv_apr_err();
	if (!ret) {
		set_vivo_ktv_state(flag);
	}

fail_cmd:
	set_vivo_ktv_cmd_state(0);
	return ret;
}

int vivo_audio_ktv_set_params(struct adm_ktv_params *param)
{
	int ret = 0;
	struct adm_cmd_set_ktv_params open;

	memcpy(&open.params, param, sizeof(struct adm_ktv_params));
	open.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	open.hdr.pkt_size = sizeof(open);
	open.hdr.src_svc = APR_SVC_ADM;
	open.hdr.src_domain = APR_DOMAIN_APPS;
	open.hdr.src_port = 0;
	open.hdr.dest_svc = APR_SVC_ADM;
	open.hdr.dest_domain = APR_DOMAIN_ADSP;
	open.hdr.dest_port = 0;
	open.hdr.token = 0;
	open.hdr.opcode = ADM_CMD_KTV_SET_PARAMS;

	atomic_set(&this_ktv.wait_stat, 0);
	ret = vivo_send_adm_apr_pkt((uint32_t *)&open);
	if (ret < 0) {
		pr_err("%s set ktv outside params fail!!\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_ktv.wait,
	                         atomic_read(&this_ktv.wait_stat),
	                         msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s wait for event timeout!!!\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = get_ktv_apr_err();

fail_cmd:
	set_vivo_ktv_cmd_state(0);
	return ret;
}

int vivo_audio_ktv_get_params(char *params, int params_length,
                              int ktv_param_id)
{
	struct adm_cmd_get_ktv_params *adm_params = NULL;
	int sz, rc = 0, i = 0, common_length = 0;
	int *params_data = (int *)params;

	common_length = sizeof(struct adm_ktv_common_param);
	sz = sizeof(struct adm_cmd_get_ktv_params);
	adm_params = kzalloc(sz, GFP_KERNEL);
	if (!adm_params) {
		pr_err("%s, adm params memory alloc failed", __func__);
		return -ENOMEM;
	}

	adm_params->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	adm_params->hdr.pkt_size = sz;
	adm_params->hdr.src_svc = APR_SVC_ADM;
	adm_params->hdr.src_domain = APR_DOMAIN_APPS;
	adm_params->hdr.src_port = 0;
	adm_params->hdr.dest_svc = APR_SVC_ADM;
	adm_params->hdr.dest_domain = APR_DOMAIN_ADSP;
	adm_params->hdr.dest_port = 0;
	adm_params->hdr.token = 0;
	adm_params->hdr.opcode = ADM_CMD_KTV_GET_PARAMS;
	adm_params->data_payload_addr_lsw = 0;
	adm_params->data_payload_addr_msw = 0;
	adm_params->mem_map_handle = 0;
	adm_params->ktv_param_id = ktv_param_id;
	adm_params->param_max_size = common_length + params_length;
	adm_params->reserved = 0;

	atomic_set(&this_ktv.wait_stat, 0);
	rc = vivo_send_adm_apr_pkt((uint32_t *)adm_params);
	if (rc < 0) {
		pr_err("%s: Failed to Get Ktv Params\n", __func__);
		rc = -EINVAL;
		goto get_param_return;
	}

	/* Wait for the callback with copp id */
	rc = wait_event_timeout(this_ktv.wait,
	                        atomic_read(&this_ktv.wait_stat),
	                        msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: get ktv params timed out\n", __func__);
		rc = -EINVAL;
		goto get_param_return;
	}

	if (params_data) {
		int length = params_length/sizeof(uint32_t);
		if (ktv_get_params[0] < length)
			length = ktv_get_params[0];
		for (i = 0; i < length; i++)
			params_data[i] = ktv_get_params[1+i];
	}

	rc = get_ktv_apr_err();
get_param_return:
	if ((ktv_param_id == KTV_TYPE_STATUS) && params_data) {
		set_vivo_ktv_state(rc ? 0 : params_data[0]);
	}

	kfree(adm_params);
	adm_params = NULL;
	set_vivo_ktv_cmd_state(0);
	return rc;
}

void copy_params_to_ucontrol(long *dst, int *src, int count)
{
	int i = 0;
	for (i = 0; i < count; i++) {
		dst[i] = src[i];
	}
}

void copy_params_from_ucontrol(int *dst, long *src, int count)
{
	int i = 0;
	for (i = 0; i < count; i++) {
		dst[i] = src[i];
	}
}

static int vivo_audio_ktv_enable_get(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
	int *kparam;
	int size_in_bytes = sizeof(int);

	kparam = (int *) kzalloc(size_in_bytes, GFP_KERNEL);
	if (!kparam) {
		pr_err("%s alloc memory failed.\n", __func__);
		return 0;
	}

	set_vivo_ktv_cmd_state(1);
	vivo_audio_ktv_get_params((char *)kparam, size_in_bytes, KTV_TYPE_STATUS);
	pr_info("%s() ktv enable %d\n", __func__, *kparam);
	copy_params_to_ucontrol(ucontrol->value.integer.value,
		kparam, size_in_bytes / sizeof(int));

	kfree(kparam);
	kparam = NULL;
	return 0;
}

static int vivo_audio_ktv_enable_put(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
	uint32_t enable_flag = 0;

	enable_flag = ucontrol->value.integer.value[0];
	pr_info("%s: enable_flag  = %d\n", __func__, enable_flag);

	if (enable_flag != get_vivo_ktv_state()) {
		set_vivo_ktv_cmd_state(1);
		vivo_audio_ktv_open(enable_flag);
	} else {
		pr_warn("%s ktv has already been %s.\n",
			__func__, enable_flag ? "opened" : "closed");
	}

	return 0;
};

/* for ktv params which not in appi module */
static int vivo_audio_ktv_param_get(struct snd_kcontrol *kcontrol,
                                    struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct adm_ktv_params *kparam;
	int size_in_bytes = sizeof(struct adm_ktv_params);

	kparam = (struct adm_ktv_params *) kzalloc(size_in_bytes, GFP_KERNEL);
	if (!kparam) {
		pr_err("%s alloc memory failed.\n", __func__);
		return 0;
	}

	if (get_vivo_ktv_state()) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_audio_ktv_get_params((char *)kparam, size_in_bytes, KTV_TYPE_PARAMS);
	} else {
		pr_err("%s() KTV Not Enabled, Operation Not allowd.\n", __func__);
		ret = -EPERM;
		goto done;
	}

	pr_info("%s: music_gain(0x%x), mix_rate(0x%x), mixed(0x%x),ears_back(0x%x)\n",
	        __func__, kparam->music_gain, kparam->mix_rate,
	        kparam->mixed, kparam->vivo_ktv_ctl);

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)kparam, size_in_bytes / sizeof(int));

done:
	kfree(kparam);
	kparam = NULL;
	return ret;
}

static int vivo_audio_ktv_param_put(struct snd_kcontrol *kcontrol,
                                    struct snd_ctl_elem_value *ucontrol)
{
	struct adm_ktv_params kparam;
	int size_in_bytes = sizeof(struct adm_ktv_params);
	int num_params = size_in_bytes / sizeof(int);
	int ret = 0;

	memset(&kparam, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&kparam,
		ucontrol->value.integer.value, num_params);
	/* kparam.mixed = !!kparam.mixed; */
	kparam.vivo_ktv_ctl = !!kparam.vivo_ktv_ctl;

	pr_info("%s: music_gain(0x%x), mix_rate(0x%x), mixed(0x%x), ears_back(0x%x)\n",
	        __func__, kparam.music_gain, kparam.mix_rate,
	        kparam.mixed, kparam.vivo_ktv_ctl);

	if (get_vivo_ktv_state()) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_audio_ktv_set_params(&kparam);
	} else {
		pr_err("%s() KTV Not Enabled, Operation Not allowd.\n", __func__);
		ret = -EPERM;
	}

	return ret;
};


/* for eq params */
static int vivo_ktv_eq_param_get(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct equalizer_params *eq;
	int size_in_bytes = sizeof(struct equalizer_params);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;
	int ret = 0 ;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_EQ_PARAMS,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	eq = (struct equalizer_params *)param_value;

	pr_info("%s: enable %d, p0(%d), p1(%d), p2(%d), p3(%d), p4(%d), "
		"p5(%d), p6(%d), p7(%d), p8(%d), p9(%d)\n",
		__func__, eq->enable, eq->params[0], eq->params[1], eq->params[2],
		eq->params[3], eq->params[4], eq->params[5], eq->params[6],
		eq->params[7], eq->params[8], eq->params[9]);

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)eq, size_in_bytes / sizeof(int));

done:
	kfree(param_value);
	param_value= NULL;
	return ret;
}

static int vivo_ktv_eq_param_put(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct equalizer_params eq;
	int i = 0, ret = 0;
	int size_in_bytes = sizeof(struct equalizer_params);
	int num_params = size_in_bytes / sizeof(int);

	memset(&eq, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&eq, ucontrol->value.integer.value, num_params);
	for (i = 0; i < num_params -1; i++) {
		if ((eq.params[i]) > 15)
			eq.params[i] = 15;
		else if (eq.params[i] < -15)
			eq.params[i] = -15;
	}

	pr_info("%s: enable %d, p0(%d), p1(%d), p2(%d), p3(%d), p4(%d), "
		"p5(%d), p6(%d), p7(%d), p8(%d), p9(%d)\n",
		__func__, eq.enable, eq.params[0], eq.params[1],
		eq.params[2], eq.params[3], eq.params[4], eq.params[5],
		eq.params[6], eq.params[7], eq.params[8], eq.params[9]);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params( (u8 *)&eq,
			PARAM_ID_KTV_EQ_PARAMS, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
};

/* for reverb params */
static int vivo_ktv_reverb_param_get(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
	struct reverb_params_vivo *reverb;
	int i = 0, ret = 0 ;
	int size_in_bytes = sizeof(struct reverb_params_vivo);
	int num_params = size_in_bytes / sizeof(int);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_REVERB_PARAMS,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	reverb = (struct reverb_params_vivo *)param_value;

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)reverb, num_params);
	for (i = 0; i < num_params; i++) {
		pr_info("%s: num[%d] = 0x%lx", __func__,
			i, ucontrol->value.integer.value[i]);
	}

done:
	kfree(param_value);
	param_value = NULL;
	return ret;
}

static int vivo_ktv_reverb_param_put(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
	struct reverb_params_vivo reverb;
	int  ret = 0;
	int size_in_bytes = sizeof(struct reverb_params_vivo);
	int num_params = size_in_bytes / sizeof(int);

	memset(&reverb, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&reverb,
		ucontrol->value.integer.value, num_params);
	reverb.enable = !!reverb.enable;

	pr_info("%s: enable %d, roomsize(0x%x), damp(0x%x), wet(0x%x), "
		"dry(0x%x), width(0x%x), gain(0x%x)\n", __func__, reverb.enable,
		reverb.roomsize, reverb.damp, reverb.wet, reverb.dry,
		reverb.width, reverb.gain);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params((u8 *)&reverb,
			PARAM_ID_KTV_REVERB_PARAMS, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
};

/* for echo params */
static int vivo_ktv_echo_param_get(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
	struct iirecho_params *echo;
	int i = 0, ret = 0 ;
	int size_in_bytes = sizeof(struct iirecho_params);
	int num_params = size_in_bytes / sizeof(int);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_ECHO_PARAMS,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	echo = (struct iirecho_params *)param_value;

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)echo, num_params);
	for (i = 0; i < num_params; i++) {
		pr_info("%s: num[%d] = 0x%lx", __func__,
			i, ucontrol->value.integer.value[i]);
	}

done:
	kfree(param_value);
	param_value = NULL;
	return ret;
}

static int vivo_ktv_echo_param_put(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
	struct iirecho_params echo;
	int  ret = 0;
	int size_in_bytes = sizeof(struct iirecho_params);
	int num_params = size_in_bytes / sizeof(int);

	memset(&echo, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&echo,
		ucontrol->value.integer.value, num_params);
	echo.enable = !!echo.enable;

	pr_info("%s: enable %d, feedback(0x%x), delay(0x%x), wet(0x%x), dry(0x%x)\n",
	        __func__, echo.enable, echo.feedback, echo.delay, echo.wet, echo.dry);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params( (u8 *)&echo,
			PARAM_ID_KTV_ECHO_PARAMS, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
};

/* for noise cancel params */
static int vivo_ktv_nc_param_get(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct noise_cancel_params *nc;
	int i = 0, ret = 0 ;
	int size_in_bytes = sizeof(struct noise_cancel_params);
	int num_params = size_in_bytes / sizeof(int);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_NOISECANCEL_PARAMS,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	nc = (struct noise_cancel_params *)param_value;

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)nc, num_params);
	for (i = 0; i < num_params; i++) {
		pr_info("%s: num[%d] = %ld\n", __func__,
			i, ucontrol->value.integer.value[i]);
	}

done:
	kfree(param_value);
	param_value = NULL;
	return ret;
}

static int vivo_ktv_nc_param_put(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct noise_cancel_params nc;
	int  ret = 0;
	int size_in_bytes = sizeof(struct noise_cancel_params);
	int num_params = size_in_bytes / sizeof(int);

	memset(&nc, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&nc,
		ucontrol->value.integer.value, num_params);
	nc.enable = !!nc.enable;

	pr_info("%s: enable %d\n", __func__, nc.enable);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params((u8 *)&nc,
			PARAM_ID_KTV_NOISECANCEL_PARAMS, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
};

/* for gramophone params */
static int vivo_ktv_gramophone_param_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct gramophone_params *gramophone;
	int i = 0, ret = 0;
	int size_in_bytes = sizeof(struct gramophone_params);
	int num_params = size_in_bytes / sizeof(int);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_GRAMOPHONE_PARAMS,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	gramophone = (struct gramophone_params *)param_value;

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)gramophone, num_params);
	for (i = 0; i < num_params; i++) {
		pr_info("%s: num[%d] = %ld\n", __func__,
			i, ucontrol->value.integer.value[i]);
	}

done:
	kfree(param_value);
	param_value = NULL;
	return ret;
}

static int vivo_ktv_gramophone_param_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct gramophone_params gramophone;
	int  ret = 0;
	int size_in_bytes = sizeof(struct gramophone_params);
	int num_params = size_in_bytes / sizeof(int);

	memset(&gramophone, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&gramophone,
		ucontrol->value.integer.value, num_params);
	gramophone.enable = !!gramophone.enable;

	pr_info("%s: enable %d\n", __func__, gramophone.enable);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params( (u8 *)&gramophone,
			PARAM_ID_KTV_GRAMOPHONE_PARAMS, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
};

/* for gramophone params */
static int vivo_audio_ktv_voice_gain_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct voice_params *voice;
	int i = 0, ret = 0;
	int size_in_bytes = sizeof(struct voice_params);
	int num_params = size_in_bytes / sizeof(int);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_VOICE_GAIN,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	voice = (struct voice_params *)param_value;

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)voice, num_params);
	for (i = 0; i < num_params; i++) {
		pr_info("%s: num[%d] = %ld\n", __func__,
			i, ucontrol->value.integer.value[i]);
	}

done:
	kfree(param_value);
	param_value = NULL;
	return ret;
}

static int vivo_audio_ktv_voice_gain_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct voice_params voice;
	int  ret = 0;
	int size_in_bytes = sizeof(struct voice_params);
	int num_params = size_in_bytes / sizeof(int);

	memset(&voice, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&voice,
		ucontrol->value.integer.value, num_params);

	pr_info("%s: voice_gain %d\n", __func__, voice.voice_gain);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params( (u8 *)&voice,
			PARAM_ID_KTV_VOICE_GAIN, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
};

/* for compressor params */
static int vivo_ktv_comp_param_get(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
	struct compressor_params *comp;
	int i = 0, ret = 0;
	int size_in_bytes = sizeof(struct compressor_params);
	int num_params = size_in_bytes / sizeof(int);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_COMPRESSOR_PARAMS,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	comp = (struct compressor_params *)param_value;

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)comp, num_params);
	for (i = 0; i < num_params; i++) {
		pr_info("%s: num[%d] = %ld\n", __func__,
			i, ucontrol->value.integer.value[i]);
	}

done:
	kfree(param_value);
	param_value = NULL;
	return ret;
}

static int vivo_ktv_comp_param_put(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
	struct compressor_params comp;
	int ret = 0;
	int size_in_bytes = sizeof(struct compressor_params);
	int num_params = size_in_bytes / sizeof(int);

	memset(&comp, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&comp,
		ucontrol->value.integer.value, num_params);
	comp.enable = !!comp.enable;

	pr_info("%s: enable %d\n", __func__, comp.enable);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params((u8 *)&comp,
			PARAM_ID_KTV_COMPRESSOR_PARAMS, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
};

/* for noise cancel params */
static int vivo_ktv_hc_param_get(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct howling_cancel_params *hc;
	int i = 0, ret = 0;
	int size_in_bytes = sizeof(struct howling_cancel_params);
	int num_params = size_in_bytes / sizeof(int);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_HC_PARAMS,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	hc = (struct howling_cancel_params *)param_value;

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)hc, num_params);
	for (i = 0; i < num_params; i++) {
		pr_info("%s: num[%d] = %ld\n", __func__,
			i, ucontrol->value.integer.value[i]);
	}

done:
	kfree(param_value);
	param_value = NULL;
	return ret;
}

static int vivo_ktv_hc_param_put(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct howling_cancel_params hc;
	int ret = 0;
	int size_in_bytes = sizeof(struct howling_cancel_params);
	int num_params = size_in_bytes / sizeof(int);

	memset(&hc, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&hc,
		ucontrol->value.integer.value, num_params);
	hc.enable = !!hc.enable;

	pr_info("%s: enable %d\n", __func__, hc.enable);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params((u8 *)&hc,
			PARAM_ID_KTV_HC_PARAMS, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
};

/* for ktv appi module enable */
static int vivo_ktv_module_enable_get(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
	struct ktv_func_enable *kf;
	int i = 0, ret = 0 ;
	int size_in_bytes = sizeof(struct ktv_func_enable);
	int num_params = size_in_bytes / sizeof(int);
	int params_length = size_in_bytes + sizeof(struct param_hdr_v3);
	char *param_value;

	param_value = kzalloc(params_length, GFP_KERNEL);
	if (!param_value) {
		pr_err("%s alloc memory failed.\n", __func__);
		return -ENOMEM;
	}

	if (get_vivo_ktv_topo_state() != AUDIO_TX_KTV_TOPOLOGY) {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
		goto done;
	}

	set_vivo_ktv_cmd_state(1);

	ret = vivo_adm_get_params(this_ktv.port_id, this_ktv.copp_idx,
		CAPI_V2_MODULE_KTV, PARAM_ID_KTV_ENABLE,
		params_length, (char *) param_value);
	if (ret) {
		pr_err("%s: get parameters failed.\n", __func__);
		goto done;
	}

	kf = (struct ktv_func_enable *)param_value;

	copy_params_to_ucontrol(ucontrol->value.integer.value,
		(int *)kf, num_params);
	for (i = 0; i < num_params; i++) {
		pr_info("%s: num[%d] = %ld\n", __func__,
			i, ucontrol->value.integer.value[i]);
	}

done:
	kfree(param_value);
	param_value = NULL;
	return ret;
}

static int vivo_ktv_module_enable_put(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
	struct ktv_func_enable kf;
	int ret = 0;
	int size_in_bytes = sizeof(struct ktv_func_enable);
	int num_params = size_in_bytes / sizeof(int);

	memset(&kf, 0, size_in_bytes);
	copy_params_from_ucontrol((int *)&kf,
		ucontrol->value.integer.value, num_params);
	kf.enable = !!kf.enable;

	pr_info("%s: enable %d\n", __func__, kf.enable);

	if (get_vivo_ktv_topo_state() == AUDIO_TX_KTV_TOPOLOGY) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_adm_send_params((u8 *)&kf,
			PARAM_ID_KTV_ENABLE, size_in_bytes);
	} else {
		pr_err("%s() KTV Not Running ,Topology is 0x%x.\n", __func__,
		       this_ktv.topology);
		ret = -EPERM;
	}

	return ret;
}

static int vivo_audio_ktv_info_get(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
	pr_info("%s\n",__func__);
	return 0;
}

static int vivo_audio_ktv_info_put(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct adm_ktv_params *kparam;
	int size_in_bytes = sizeof(struct adm_ktv_params);

	pr_info("%s--------------Info---------------\n", __func__);
	pr_info("%s---KTV State   is %s---\n", __func__, get_vivo_ktv_state() ? "opened" : "closed");
	pr_info("%s---Topology id is 0x%x---\n", __func__, get_vivo_ktv_topo_state());
	pr_info("%s---Tx port  id is 0x%x---\n", __func__, this_ktv.port_id);
	pr_info("%s---Copp     id is 0x%x---\n", __func__, this_ktv.copp_idx);

	kparam = (struct adm_ktv_params *) kzalloc(size_in_bytes, GFP_KERNEL);
	if (!kparam) {
		pr_err("%s alloc memory failed.\n", __func__);
		return 0;
	}

	if (get_vivo_ktv_state()) {
		set_vivo_ktv_cmd_state(1);
		ret = vivo_audio_ktv_get_params((char *)kparam,
			size_in_bytes, KTV_TYPE_PARAMS);
		pr_info("%s---Ears_back ctl is 0x%x---\n",
			__func__, kparam->vivo_ktv_ctl);
	}

	kfree(kparam);
	kparam = NULL;
	return ret;
}

static const char *const vivo_audio_ktv_enable_text[] =
	{"off", "on"};
static const struct soc_enum vivo_audio_ktv_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, vivo_audio_ktv_enable_text),
};

static const struct snd_kcontrol_new vivo_audio_ktv_controls[] = {
	SOC_ENUM_EXT("VIVO AUDIO KTV ENABLE", vivo_audio_ktv_enum[0],
			vivo_audio_ktv_enable_get,
			vivo_audio_ktv_enable_put),
	SOC_SINGLE_EXT("KTV INFO", SND_SOC_NOPM, 0, 1, 0,
			vivo_audio_ktv_info_get,
			vivo_audio_ktv_info_put),
	SOC_SINGLE_MULTI_EXT("KTV EQ PARAMS", SND_SOC_NOPM, 0, 15, 0, 11,
			vivo_ktv_eq_param_get,
			vivo_ktv_eq_param_put),
	SOC_SINGLE_MULTI_EXT("KTV Reverb PARAMS", SND_SOC_NOPM, 0, 0x2000, 0, 7,
			vivo_ktv_reverb_param_get,
			vivo_ktv_reverb_param_put),
	SOC_SINGLE_MULTI_EXT("KTV ECHO PARAMS", SND_SOC_NOPM, 0, 0x2000, 0, 5,
			vivo_ktv_echo_param_get,
			vivo_ktv_echo_param_put),
	SOC_SINGLE_EXT("KTV Noise Cancel PARAMS", SND_SOC_NOPM, 0, 1, 0,
			vivo_ktv_nc_param_get,
			vivo_ktv_nc_param_put),
	SOC_SINGLE_EXT("KTV Gramophone PARAMS", SND_SOC_NOPM, 0, 1, 0,
			vivo_ktv_gramophone_param_get,
			vivo_ktv_gramophone_param_put),
	SOC_SINGLE_EXT("KTV VOICE GAIN", SND_SOC_NOPM, 0, 0x4000, 0,
			vivo_audio_ktv_voice_gain_get,
			vivo_audio_ktv_voice_gain_put),
	SOC_SINGLE_EXT("KTV Compressor PARAMS", SND_SOC_NOPM, 0, 1, 0,
			vivo_ktv_comp_param_get,
			vivo_ktv_comp_param_put),
	SOC_SINGLE_EXT("KTV Howling Cancel PARAMS", SND_SOC_NOPM, 0, 1, 0,
			vivo_ktv_hc_param_get,
			vivo_ktv_hc_param_put),
	SOC_SINGLE_EXT("KTV Module Enable", SND_SOC_NOPM, 0, 1, 0,
			vivo_ktv_module_enable_get,
			vivo_ktv_module_enable_put),
	SOC_SINGLE_MULTI_EXT("KTV PARAMS OUT Of Module", SND_SOC_NOPM, 0, 0x7fffffff, 0, 4,
			vivo_audio_ktv_param_get,
			vivo_audio_ktv_param_put),
};

void vivo_ktv_add_controls(struct snd_soc_component *component)
{
	snd_soc_add_component_controls(component, vivo_audio_ktv_controls,
				      ARRAY_SIZE(vivo_audio_ktv_controls));
}
EXPORT_SYMBOL(vivo_ktv_add_controls);

void vivo_ktv_init(void)
{
	init_waitqueue_head(&this_ktv.wait);
	vivo_ktv_reset();
	memset(&ktv_get_params, 0, KTV_PARAMS_SIZE_MAX * sizeof(int));
}
