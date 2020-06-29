#include <sound/soc.h>
#include <linux/syscalls.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include "algo/inc/tas25xx-calib.h"
#include "algo/inc/tas_smart_amp_v2.h"
#include "tas25xx-algo-bin-utils.h"

#define CODEC_CONTROL		1

static int32_t g_fmt;
static int s_tas_smartamp_bypass;
static struct mutex smartpa_algo_lock;
static struct device *s_device;

int tas_smartamp_add_algo_controls_debugfs(struct snd_soc_codec *codec,
	int number_of_channels);
void tas_smartamp_remove_algo_controls_debugfs(struct snd_soc_codec *codec);

void tas25xx_set_algo_bypass(int bypass)
{
	s_tas_smartamp_bypass = bypass;
}

int tas25xx_get_algo_bypass(void)
{
	return s_tas_smartamp_bypass;
}

int get_iv_vbat_format(void)
{
	return g_fmt;
}

void tas25xx_algo_set_device (struct device *dev)
{
	s_device = dev;
}

struct device *tas25xx_algo_get_device(void)
{
	return s_device;
}

bool tas25xx_set_iv_bit_fomat(int iv_data_with, int vbat, int update_now)
{
	int32_t param_id;
	int32_t ret;
	bool success;

	if ((vbat == 1) && (iv_data_with == 12))
		g_fmt = IV_SENSE_FORMAT_12_BIT_WITH_8BIT_VBAT;
	else if ((vbat == 1) && (iv_data_with == 8))
		g_fmt = IV_SENSE_FORMAT_8_BIT_WITH_8BIT_VBAT;
	else if ((vbat == 1) && (iv_data_with == 16))
		g_fmt = IV_SENSE_FORMAT_16_BIT_WITH_8BIT_VBAT;
	else
		g_fmt = IV_SENSE_FORMAT_NO_VBAT;

	success = true;

	if (update_now) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_IV_VBAT_FMT, 1, CHANNEL0);
		pr_info("TI-SmartPA: %s: Sending IV,Vbat format %d\n",
			__func__, g_fmt);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&g_fmt, param_id,
				TAS_SET_PARAM, sizeof(uint32_t),
				AFE_SMARTAMP_MODULE_RX);
		if (ret < 0) {
			pr_err("TI-SmartPA: %s: Failed to set config\n",
				__func__);
			success = false;
		}

	}

	return success;
}
EXPORT_SYMBOL(tas25xx_set_iv_bit_fomat);


static int tas25xx_smartamp_get_set(u8 *user_data, uint32_t param_id,
		uint8_t get_set, uint32_t length, uint32_t module_id)
{
	int ret = 0;

	switch (get_set) {
	case TAS_SET_PARAM:
		if (s_tas_smartamp_bypass) {
			pr_err("TI-SmartPA: %s: SmartAmp is bypassed no control set\n",
				__func__);
			goto fail_cmd;
		}
		ret = afe_tas_smartamp_set_calib_data(module_id, param_id,
			length, user_data);
		break;

	case TAS_GET_PARAM:
		if (!s_tas_smartamp_bypass) {
			memset(user_data, 0, length);
			ret = afe_tas_smartamp_get_calib_data(module_id,
				param_id, length, user_data);
		}
		break;

	default:
		goto fail_cmd;
	}

fail_cmd:
	return ret;
}

/*Wrapper arround set/get parameter,
 *all set/get commands pass through this wrapper
 */
int tas25xx_smartamp_algo_ctrl(u8 *user_data, uint32_t param_id,
		uint8_t get_set, uint32_t length, uint32_t module_id)
{
	int ret = 0;

	mutex_lock(&smartpa_algo_lock);
	ret = tas25xx_smartamp_get_set(user_data, param_id, get_set,
		length, module_id);
	mutex_unlock(&smartpa_algo_lock);
	return ret;
}
EXPORT_SYMBOL(tas25xx_smartamp_algo_ctrl);


#if CODEC_CONTROL
void tas_smartamp_add_algo_controls(struct snd_soc_codec *codec, struct device *dev, int number_of_channels)
{
	pr_info("TI-SmartPA: %s: Adding smartamp controls\n", __func__);

	g_fmt = 0;

	mutex_init(&smartpa_algo_lock);

#if IS_ENABLED(CONFIG_TISA_DEBUGFS_INTF)
	pr_err("TI-SmartPA: %s: Adding debugfs controls\n", __func__);
	tas_smartamp_add_algo_controls_debugfs(codec, number_of_channels);
#endif
#ifdef CONFIG_TISA_BIN_INTF
	pr_err("TI-SmartPA: %s: Adding bin intf controls\n", __func__);
	tas_smartamp_add_codec_mixer_controls(codec);
#endif

#ifdef CONFIG_TISA_KBIN_INTF
	tas_smartamp_add_codec_mixer_controls (codec);
	tas25xx_algo_set_device (dev);
	bin_file_set_device (dev);
	bin_file_parse_init();

	pr_err("TI-SmartPA: %s: Initialising kbin file done\n", __func__);
#endif

	tas_calib_init();
}
EXPORT_SYMBOL(tas_smartamp_add_algo_controls);
#else
void tas_smartamp_add_algo_controls_for_platform(
	struct snd_soc_platform *platform)
{
	pr_info("TI-SmartPA: %s: Adding smartamp controls\n", __func__);

	g_fmt = 0;

#ifdef CONFIG_TISA_BIN_INTF
	tas_smartamp_add_platform_mixer_controls(platform);
#endif

	tas_calib_init();
}
EXPORT_SYMBOL(tas_smartamp_add_algo_controls_for_platform);
#endif

void tas_smartamp_remove_algo_controls(struct snd_soc_codec *codec)
{
#ifdef CONFIG_TISA_DEBUGFS_INTF
	tas_smartamp_remove_algo_controls_debugfs(codec);
#endif
#ifdef CONFIG_TISA_KBIN_INTF
	tas_smartamp_kbin_deinitalize ();
	tas25xx_algo_set_device(NULL);
	bin_file_set_device(NULL);
	bin_file_parse_deinit();
	mutex_destroy(&smartpa_algo_lock);
#endif
	tas_calib_exit();
}
EXPORT_SYMBOL(tas_smartamp_remove_algo_controls);
