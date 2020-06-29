#ifndef __TAS_SMART_AMP__
#define __TAS_SMART_AMP__

#include <linux/types.h>
#include <sound/soc.h>
#include <linux/delay.h>

#ifdef CONFIG_PLATFORM_QCOM
#include <dsp/tas_qualcomm.h>
#endif /*CONFIG_PLATFORM_QCOM*/

#define CONFIG_TAS25XX_ALGO_STEREO
#define CONFIG_SET_RE_IN_KERNEL

/*FIXME*/
/*update this value at OEM/ODM*/
#pragma message("########### FIXME ################")
/*#define SMARTAMP_SPEAKER_CALIBDATA_FILE "/mnt/vendor/persist/audio/tas25xx_calib.bin"*/
#define SMARTAMP_SPEAKER_CALIBDATA_FILE "/persist/audio/smartamp_calib.bin"

#define MAX_DSP_PARAM_INDEX             2048
#define MAX_CHANNELS					2

#define TAS_GET_PARAM           1
#define TAS_SET_PARAM           0

#define CHANNEL0        1
#define CHANNEL1        2

#define TRUE            1
#define FALSE           0

#define TAS_DSP_SWAP_IDX	 	3

#define TAS_SA_GET_F0          3810
#define TAS_SA_GET_Q           3811
#define TAS_SA_GET_TV          3812
#define TAS_SA_GET_RE          3813
#define TAS_SA_CALIB_INIT      3814
#define TAS_SA_CALIB_DEINIT    3815
#define TAS_SA_SET_RE          3816
#define TAS_SA_F0_TEST_INIT    3817
#define TAS_SA_F0_TEST_DEINIT  3818
#define TAS_SA_SET_PROFILE     3819
#define TAS_SA_GET_STATUS      3821
#define TAS_SA_SET_SPKID       3822
#define TAS_SA_SET_TCAL        3823

#define TAS_SA_IV_VBAT_FMT     3825
/*Added for DC Detection*/
#define CAPI_V2_TAS_SA_DC_DETECT	0x40404040

#define CALIB_START             1
#define CALIB_STOP              2
#define TEST_START              3
#define TEST_STOP               4

#define SLAVE1          0x98
#define SLAVE2          0x9A
#define SLAVE3          0x9C
#define SLAVE4          0x9E

#define CHANNEL0                           1
#define CHANNEL1                           2

#define TAS_SA_IS_SPL_IDX(X)		((((X) >= 3810) && ((X) < 3899)) ? 1 : 0)
#define TAS_CALC_PARAM_IDX(I, LEN, CH)   ((I) | (LEN << 16) | (CH << 28))
#define AFE_SA_IS_SPL_IDX(X)		TAS_SA_IS_SPL_IDX(X)


typedef enum {
	IV_SENSE_FORMAT_NO_VBAT = 0,
	IV_SENSE_FORMAT_12_BIT_WITH_8BIT_VBAT = 1,
	IV_SENSE_FORMAT_8_BIT_WITH_8BIT_VBAT = 2,
	IV_SENSE_FORMAT_16_BIT_WITH_8BIT_VBAT = 3,
} ti_smartamp_iv_vbat_format_t;

/*
 * List all the other profiles other than none and calibration.
 */
#define TAS_ALGO_PROFILE_LIST          "MUSIC", "VOICE", "VOIP", "RINGTONE"

int tas25xx_smartamp_algo_ctrl(u8 *user_data,
		uint32_t param_id, uint8_t get_set,
		uint32_t length, uint32_t module_id);
void tas_smartamp_add_algo_controls(struct snd_soc_codec *codec, struct device *dev, int number_of_channels);
void tas_smartamp_remove_algo_controls(struct snd_soc_codec *codec);
bool tas25xx_set_iv_bit_fomat(int iv_data_with, int vbat, int update_now);

/* bypass set and get */
int tas25xx_get_algo_bypass(void);
void tas25xx_set_algo_bypass(int bypass);

/**/
int get_iv_vbat_format(void);

void tas_smartamp_add_codec_mixer_controls(struct snd_soc_codec *codec);

#if IS_ENABLED(CONFIG_TISA_DEBUGFS_INTF)
int tas25xx_parse_algo_dt(struct device_node *np);
int smaramp_set_i2c_client(struct i2c_client *p_client);
#endif /*CONFIG_TISA_DEBUGFS_INTF*/

#if IS_ENABLED(CONFIG_TISA_KBIN_INTF)
void tas25xx_algo_set_active (void);
void tas25xx_algo_set_inactive (void);
void tas_smartamp_kbin_deinitalize (void);
#endif /* CONFIG_TISA_KBIN_INTF */

#endif /*__TAS_SMART_AMP__*/
