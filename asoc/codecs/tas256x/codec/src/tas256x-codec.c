/*
 * =============================================================================
 * Copyright (c) 2016  Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
 *
 * File:
 *     tas256x-codec.c
 *
 * Description:
 *     ALSA SoC driver for Texas Instruments TAS256X High Performance 4W Smart
 *     Amplifier
 *
 * =============================================================================
 */

#ifdef CONFIG_TAS256X_CODEC
#define DEBUG 5
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/version.h>

#include "tas256x/inc/tas256x.h"
#include "tas256x/inc/tas256x-device.h"
#include "src/tas256x-logic.h"
#include "codec/inc/tas256x-regmap.h"
#ifdef CONFIG_TAS25XX_ALGO
#include "algo/inc/tas_smart_amp_v2.h"
#include "algo/inc/tas25xx-calib.h"
#endif /*CONFIG_TAS25XX_ALGO*/

#define TAS256X_MDELAY 0xFFFFFFFE
#define TAS256X_MSLEEP 0xFFFFFFFD
#define TAS256X_IVSENSER_ENABLE  1
#define TAS256X_IVSENSER_DISABLE 0
/* #define TAS2558_CODEC */

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static unsigned int tas256x_codec_read(struct snd_soc_component *codec,
		unsigned int reg)
{
	unsigned int value = 0;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	int ret = 0;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	switch (reg) {
	case TAS256X_LEFT_SWITCH:
		value = p_tas256x->devs[0]->spk_control;
		break;
	case TAS256X_RIGHT_SWITCH:
		value = p_tas256x->devs[1]->spk_control;
		break;
	case RX_SCFG_LEFT:
		value = p_tas256x->devs[0]->rx_cfg;
		break;
	case RX_SCFG_RIGHT:
		value = p_tas256x->devs[1]->rx_cfg;
		break;
	default:
		ret = p_tas256x->read(p_tas256x, channel_left, reg,
			&value);
		break;
	}

	dev_dbg(plat_data->dev, "%s, reg=%d, value=%d", __func__, reg, value);

	if (ret == 0)
		return value;
	else
		return ret;
}
#else
static unsigned int tas256x_codec_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	unsigned int value = 0;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	switch (reg) {
	case TAS256X_LEFT_SWITCH:
		value = p_tas256x->devs[0]->spk_control;
		break;
	case TAS256X_RIGHT_SWITCH:
		value = p_tas256x->devs[1]->spk_control;
		break;
	case RX_SCFG_LEFT:
		value = p_tas256x->devs[0]->rx_cfg;
		break;
	case RX_SCFG_RIGHT:
		value = p_tas256x->devs[1]->rx_cfg;
		break;
	default:
		ret = p_tas256x->read(p_tas256x, channel_left, reg,
			&value);
		break;
	}

	dev_dbg(plat_data->dev, "%s, reg=%d, value=%d", __func__, reg, value);

	if (ret == 0)
		return value;
	else
		return ret;

	return value;
}
#endif

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int tas256x_codec_write(struct snd_soc_component *codec,
				unsigned int reg, unsigned int value)
{
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	dev_dbg(plat_data->dev, "%s: %d, %d", __func__, reg, value);

	switch (reg) {
	case TAS256X_LEFT_SWITCH:
		p_tas256x->devs[0]->spk_control = value;
		break;
	case TAS256X_RIGHT_SWITCH:
		p_tas256x->devs[1]->spk_control = value;
		break;
	case RX_SCFG_LEFT:
		ret = tas256x_update_rx_cfg(p_tas256x, value,
			channel_left);
		break;
	case RX_SCFG_RIGHT:
		ret = tas256x_update_rx_cfg(p_tas256x, value,
			channel_right);
		break;
	default:
		ret = p_tas256x->write(p_tas256x, channel_both,
			reg, value);
		break;
	}

	return ret;
}
#else
static int tas256x_codec_write(struct snd_soc_codec *codec,
				unsigned int reg, unsigned int value)
{
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	dev_dbg(plat_data->dev, "%s: %d, %d", __func__, reg, value);

	switch (reg) {
	case TAS256X_LEFT_SWITCH:
		p_tas256x->devs[0]->spk_control = value;
		break;
	case TAS256X_RIGHT_SWITCH:
		p_tas256x->devs[1]->spk_control = value;
		break;
	case RX_SCFG_LEFT:
		ret = tas256x_update_rx_cfg(p_tas256x, value,
			channel_left);
		break;
	case RX_SCFG_RIGHT:
		ret = tas256x_update_rx_cfg(p_tas256x, value,
			channel_right);
		break;
	default:
		ret = p_tas256x->write(p_tas256x, channel_both,
			reg, value);
		break;
	}

	return ret;
}
#endif

#ifdef CODEC_PM
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int tas256x_codec_suspend(struct snd_soc_component *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	mutex_lock(&p_tas256x->codec_lock);

	dev_dbg(plat_data->dev, "%s\n", __func__);
	plat_data->runtime_suspend(p_tas256x);

	mutex_unlock(&p_tas256x->codec_lock);
	return ret;
}

static int tas256x_codec_resume(struct snd_soc_component *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	mutex_lock(&p_tas256x->codec_lock);

	dev_dbg(plat_data->dev, "%s\n", __func__);
	plat_data->runtime_resume(p_tas256x);

	mutex_unlock(&p_tas256x->codec_lock);
	return ret;
}
#else
static int tas256x_codec_suspend(struct snd_soc_codec *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	mutex_lock(&p_tas256x->codec_lock);

	dev_dbg(plat_data->dev, "%s\n", __func__);
	plat_data->runtime_suspend(p_tas256x);

	mutex_unlock(&p_tas256x->codec_lock);
	return ret;
}

static int tas256x_codec_resume(struct snd_soc_codec *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	mutex_lock(&p_tas256x->codec_lock);

	dev_dbg(plat_data->dev, "%s\n", __func__);
	plat_data->runtime_resume(p_tas256x);

	mutex_unlock(&p_tas256x->codec_lock);
	return ret;
}
#endif
#endif

static int tas256x_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_dapm_to_component(w->dapm);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(plat_data->dev, "SND_SOC_DAPM_POST_PMU\n");
		break;
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(plat_data->dev, "SND_SOC_DAPM_PRE_PMD\n");
		break;
	}

	return 0;
}

static const char * const tas256x_ASI1_src[] = {
	"I2C offset", "Left", "Right", "LeftRightDiv2",
};

static SOC_ENUM_SINGLE_DECL(tas2562_ASI1_src_left_enum, RX_SCFG_LEFT, 0,
			    tas256x_ASI1_src);
static SOC_ENUM_SINGLE_DECL(tas2562_ASI1_src_right_enum, RX_SCFG_RIGHT, 0,
			    tas256x_ASI1_src);

static const struct snd_kcontrol_new dapm_switch_left =
	SOC_DAPM_SINGLE("Switch", TAS256X_LEFT_SWITCH, 0, 1, 0);
static const struct snd_kcontrol_new dapm_switch_right =
	SOC_DAPM_SINGLE("Switch", TAS256X_RIGHT_SWITCH, 0, 1, 0);
static const struct snd_kcontrol_new tas256x_asi1_left_mux =
	SOC_DAPM_ENUM("Mux", tas2562_ASI1_src_left_enum);
static const struct snd_kcontrol_new tas256x_asi1_right_mux =
	SOC_DAPM_ENUM("Mux", tas2562_ASI1_src_right_enum);

static const struct snd_soc_dapm_widget tas256x_dapm_widgets_stereo[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SWITCH("TAS256X ASI Left", SND_SOC_NOPM, 0, 0,
		&dapm_switch_left),
	SND_SOC_DAPM_SWITCH("TAS256X ASI Right", SND_SOC_NOPM, 0, 0,
		&dapm_switch_right),
	SND_SOC_DAPM_MUX("TAS256X ASI1 SEL LEFT", SND_SOC_NOPM, 0, 0,
		&tas256x_asi1_left_mux),
	SND_SOC_DAPM_MUX("TAS256X ASI1 SEL RIGHT", SND_SOC_NOPM, 0, 0,
		&tas256x_asi1_right_mux),
	SND_SOC_DAPM_AIF_OUT("Voltage Sense", "ASI1 Capture",  1,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("Current Sense", "ASI1 Capture",  0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC1", NULL, SND_SOC_NOPM, 0, 0, tas256x_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("DAC2", NULL, SND_SOC_NOPM, 0, 0, tas256x_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT1"),
	SND_SOC_DAPM_OUTPUT("OUT2"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON")
};

static const struct snd_soc_dapm_route tas256x_audio_map_stereo[] = {
	{"TAS256X ASI1 SEL LEFT", "Left", "ASI1"},
	{"TAS256X ASI1 SEL LEFT", "Right", "ASI1"},
	{"TAS256X ASI1 SEL LEFT", "LeftRightDiv2", "ASI1"},
	{"TAS256X ASI1 SEL LEFT", "I2C offset", "ASI1"},
	{"TAS256X ASI1 SEL RIGHT", "Left", "ASI1"},
	{"TAS256X ASI1 SEL RIGHT", "Right", "ASI1"},
	{"TAS256X ASI1 SEL RIGHT", "LeftRightDiv2", "ASI1"},
	{"TAS256X ASI1 SEL RIGHT", "I2C offset", "ASI1"},
	{"DAC1", NULL, "TAS256X ASI1 SEL LEFT"},
	{"DAC2", NULL, "TAS256X ASI1 SEL RIGHT"},
	{"TAS256X ASI Left", "Switch", "DAC1"},
	{"TAS256X ASI Right", "Switch", "DAC2"},
	{"OUT1", NULL, "TAS256X ASI Left"},
	{"OUT2", NULL, "TAS256X ASI Right"},
	{"Voltage Sense", NULL, "VMON"},
	{"Current Sense", NULL, "IMON"}
};

static const struct snd_soc_dapm_widget tas256x_dapm_widgets_mono[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SWITCH("TAS256X ASI", SND_SOC_NOPM, 0, 0,
		&dapm_switch_left),
	SND_SOC_DAPM_MUX("TAS256X ASI1 SEL", SND_SOC_NOPM, 0, 0,
		&tas256x_asi1_left_mux),
	SND_SOC_DAPM_AIF_OUT("Voltage Sense", "ASI1 Capture",  0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("Current Sense", "ASI1 Capture",  0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0, tas256x_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON")
};

static const struct snd_soc_dapm_route tas256x_audio_map_mono[] = {
	{"TAS256X ASI1 SEL", "Left", "ASI1"},
	{"TAS256X ASI1 SEL", "Right", "ASI1"},
	{"TAS256X ASI1 SEL", "LeftRightDiv2", "ASI1"},
	{"TAS256X ASI1 SEL", "I2C offset", "ASI1"},
	{"DAC", NULL, "TAS256X ASI1 SEL"},
	{"TAS256X ASI", "Switch", "DAC"},
	{"OUT", NULL, "TAS256X ASI"},
	{"Voltage Sense", NULL, "VMON"},
	{"Current Sense", NULL, "IMON"}
};

static int tas256x_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = dai->component;
	struct tas256x_priv *p_tas256x
			= snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;
	int bitwidth = 16;
	int n_result = 0;

	dev_dbg(plat_data->dev, "%s, stream %s format: %d\n", __func__,
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? ("Playback") : ("Capture"),
		params_format(params));

	mutex_lock(&p_tas256x->codec_lock);

	if (p_tas256x->mn_fmt_mode != 2) {
		p_tas256x->mn_fmt_mode = 1;
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			bitwidth = 16;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			bitwidth = 24;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			bitwidth = 32;
			break;
		}

		n_result = tas256x_set_bitwidth(p_tas256x,
				bitwidth, substream->stream);
		if (n_result < 0) {
			dev_info(plat_data->dev, "set bitwidth failed, %d\n",
				n_result);
			goto ret;
		}
	}

	dev_info(plat_data->dev, "%s, stream %s sample rate: %d\n", __func__,
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? ("Playback") : ("Capture"),
		params_rate(params));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		n_result = tas256x_set_samplerate(p_tas256x,
			params_rate(params), channel_both);

ret:
	mutex_unlock(&p_tas256x->codec_lock);
	return n_result;
}

static int tas256x_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = dai->component;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;
	u8 tdm_rx_start_slot = 0, asi_cfg_1 = 0, asi_cfg_2 = 0;

	dev_dbg(plat_data->dev, "%s, format=0x%x\n", __func__, fmt);

	p_tas256x->mn_fmt = 1;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		asi_cfg_1 = 0x00;
		asi_cfg_2 = 0x00;
		break;
	default:
		dev_err(plat_data->dev, "ASI format master is not found\n");
		ret = -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		dev_info(plat_data->dev, "INV format: NBNF\n");
		asi_cfg_1 |= 1;
		asi_cfg_2 |= 1;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dev_info(plat_data->dev, "INV format: IBNF\n");
		asi_cfg_1 |= 0;
		asi_cfg_2 |= 1;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dev_info(plat_data->dev, "INV format: NBIF\n");
		asi_cfg_1 |= 1;
		asi_cfg_2 |= 0;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		dev_info(plat_data->dev, "INV format: IBIF\n");
		asi_cfg_1 |= 0;
		asi_cfg_2 |= 0;
		break;
	default:
		dev_err(plat_data->dev, "ASI format Inverse is not found\n");
		ret = -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case (SND_SOC_DAIFMT_I2S):
		tdm_rx_start_slot = 1;
		dev_info(plat_data->dev, " SND_SOC_DAIFMT_I2S tdm_rx_start_slot = 1\n");
		break;
	case (SND_SOC_DAIFMT_DSP_A):
		tdm_rx_start_slot = 1;
		dev_info(plat_data->dev, "SND_SOC_DAIFMT_DSP_A tdm_rx_start_slot =1\n");
		break;
	case (SND_SOC_DAIFMT_DSP_B):
		tdm_rx_start_slot = 0;
		dev_info(plat_data->dev, "SND_SOC_DAIFMT_DSP_B tdm_rx_start_slot = 0\n");
		break;
	case (SND_SOC_DAIFMT_LEFT_J):
		tdm_rx_start_slot = 0;
		dev_info(plat_data->dev, "SND_SOC_DAIFMT_LEFT_J tdm_rx_start_slot = 0\n");
		break;
	default:
	dev_err(plat_data->dev, "DAI Format is not found, fmt=0x%x\n", fmt);
	ret = -EINVAL;
		break;
	}

	ret = tas256x_rx_set_start_slot(p_tas256x,
		tdm_rx_start_slot, channel_both);
	if (ret)
		goto end;

	ret = tas256x_rx_set_edge(p_tas256x,
		asi_cfg_1, channel_both);
	if (ret)
		goto end;

	ret = tas256x_rx_set_frame_start(p_tas256x,
		asi_cfg_2, channel_both);
	if (ret)
		goto end;

end:
	return ret;
}

static int tas256x_set_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	int ret = 0;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = dai->component;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;

	dev_dbg(plat_data->dev, "%s, tx_mask:%d, rx_mask:%d",
		__func__, tx_mask, rx_mask);
	dev_dbg(plat_data->dev, "%s, slots:%d,slot_width:%d",
		__func__, slots, slot_width);

	if (rx_mask) {
		p_tas256x->mn_fmt_mode = 2; /*TDM Mode*/
		ret = tas256x_set_tdm_rx_slot(p_tas256x, slots, slot_width);
	} else if (tx_mask) {
		p_tas256x->mn_fmt_mode = 2;
		ret = tas256x_set_tdm_tx_slot(p_tas256x, slots, slot_width);
	} else {
		dev_err(plat_data->dev, "%s, Invalid Mask",
				__func__);
		p_tas256x->mn_fmt_mode = 0;
	}

	return ret;
}

static int tas256x_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = dai->component;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	dev_dbg(plat_data->dev, "%s, stream %s mute %d\n", __func__,
		(stream == SNDRV_PCM_STREAM_PLAYBACK) ? ("Playback") : ("Capture"),
		mute);
	mutex_lock(&p_tas256x->codec_lock);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (mute)
			tas256x_set_power_state(p_tas256x,
				TAS256X_POWER_SHUTDOWN);
		else
			tas256x_set_power_state(p_tas256x,
				TAS256X_POWER_ACTIVE);
	}

	mutex_unlock(&p_tas256x->codec_lock);
	return 0;
}

static struct snd_soc_dai_ops tas256x_dai_ops = {
	.hw_params  = tas256x_hw_params,
	.set_fmt    = tas256x_set_dai_fmt,
	.set_tdm_slot = tas256x_set_dai_tdm_slot,
	.mute_stream = tas256x_mute_stream,
};

#define TAS256X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
						SNDRV_PCM_FMTBIT_S20_3LE |\
						SNDRV_PCM_FMTBIT_S24_LE |\
						SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver tas256x_dai_driver[] = {
	{
		.name = "tas256x ASI1",
		.id = 0,
		.playback = {
			.stream_name    = "ASI1 Playback",
			.channels_min   = 1,
			.channels_max   = 8,
			.rates      = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS256X_FORMATS,
		},
		.capture = {
			.stream_name    = "ASI1 Capture",
			.channels_min   = 1,
			.channels_max   = 8,
			.rates          = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS256X_FORMATS,
		},
		.ops = &tas256x_dai_ops,
		.symmetric_rates = 1,
	},
};

/*Generic Control-1: IV Sense enable*/
static char const *iv_enable_text[] = {"Off", "On"};
static const struct soc_enum tas256x_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(iv_enable_text), iv_enable_text),
};

static int tas256xiv_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec
				= snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tas256x_priv *p_tas256x = NULL;
	int iv_enable = 0, n_result = 0;
	struct linux_platform *plat_data = NULL;

	if (codec == NULL) {
		pr_err("%s:codec is NULL\n", __func__);
		return 0;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return 0;
	}
	plat_data = (struct linux_platform *) p_tas256x->platform_data;

	iv_enable = ucontrol->value.integer.value[0];

	n_result = tas256x_iv_sense_enable_set(p_tas256x, iv_enable,
		channel_both);

	pr_debug("%s: tas256x->iv_enable = %d\n", __func__,
		p_tas256x->iv_enable);

	return n_result;
}

static int tas256xiv_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tas256x_priv *p_tas256x = NULL;
	struct linux_platform *plat_data = NULL;

	if (codec == NULL) {
		pr_err("%s:codec is NULL\n", __func__);
		return 0;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return 0;
	}

	plat_data = (struct linux_platform *) p_tas256x->platform_data;

	ucontrol->value.integer.value[0] =
		tas256x_iv_sense_enable_get(p_tas256x, channel_left);
	p_tas256x->iv_enable = ucontrol->value.integer.value[0];

	dev_info(plat_data->dev, "p_tas256x->iv_enable %d\n",
		p_tas256x->iv_enable);

	return 0;
}

static const struct snd_kcontrol_new tas256x_controls[] = {
SOC_ENUM_EXT("TAS256X IVSENSE ENABLE", tas256x_enum[0],
			tas256xiv_get, tas256xiv_put),
};

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int tas256x_codec_probe(struct snd_soc_component *codec)
{
	int ret, i;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	if (plat_data)
		plat_data->codec = codec;

	ret = snd_soc_add_component_controls(codec, tas256x_controls,
					 ARRAY_SIZE(tas256x_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, err %d\n",
			__func__, ret);
		return ret;
	}

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->dev_ops.tas_probe)
			ret |= (p_tas256x->devs[i]->dev_ops.tas_probe)(p_tas256x, codec, i+1);
	}

	/* Generic Probe */
	ret = tas256x_probe(p_tas256x);
	dev_dbg(plat_data->dev, "%s\n", __func__);

	return 0;
}
#else
static int tas256x_codec_probe(struct snd_soc_codec *codec)
{
	int ret, i;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	if (plat_data)
		plat_data->codec = codec;

	dev_info(plat_data->dev, "Driver Tag: %s\n", TAS256X_DRIVER_TAG);
	ret = snd_soc_add_codec_controls(codec, tas256x_controls,
					 ARRAY_SIZE(tas256x_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, err %d\n",
			__func__, ret);
		return ret;
	}

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->dev_ops.tas_probe)
			ret |= (p_tas256x->devs[i]->dev_ops.tas_probe)(p_tas256x, codec, i+1);
	}
	/* Generic Probe */
	ret = tas256x_probe(p_tas256x);
	dev_dbg(plat_data->dev, "%s\n", __func__);

	return ret;
}
#endif

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static void tas256x_codec_remove(struct snd_soc_component *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);

	tas256x_remove(p_tas256x);
}
#else
static int tas256x_codec_remove(struct snd_soc_codec *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);

	tas256x_remove(p_tas256x);
	return 0;
}
#endif

/*snd control-1: SmartPA System Mute(Master) Control*/
static int tas256x_system_mute_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec
					= snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	pValue->value.integer.value[0] = p_tas256x->mb_mute;
	dev_dbg(plat_data->dev, "%s = %d\n",
		__func__, p_tas256x->mb_mute);

	return 0;
}

static int tas256x_system_mute_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int mb_mute = pValue->value.integer.value[0];

	dev_dbg(plat_data->dev, "%s = %d\n", __func__, mb_mute);

	p_tas256x->mb_mute = !!mb_mute;

	return 0;
}

/*snd control-2: SmartPA Mute Control*/
static int tas256x_mute_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	pValue->value.integer.value[0] = p_tas256x->mb_mute;

	if ((p_tas256x->mb_power_up == true) &&
		(p_tas256x->mn_power_state == TAS256X_POWER_ACTIVE))
		pValue->value.integer.value[0] = 0;
	else
		pValue->value.integer.value[0] = 1;

	dev_dbg(plat_data->dev, "%s = %ld\n",
		__func__, pValue->value.integer.value[0]);

	return 0;
}

static int tas256x_mute_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int mute = pValue->value.integer.value[0];

	dev_dbg(plat_data->dev, "%s, %d\n", __func__, mute);
	mutex_lock(&p_tas256x->codec_lock);

	if (mute)
		tas256x_set_power_state(p_tas256x, TAS256X_POWER_MUTE);
	else
		tas256x_set_power_state(p_tas256x, TAS256X_POWER_ACTIVE);

	mutex_unlock(&p_tas256x->codec_lock);
	return 0;
}

/*snd control-3: DAC Mute Control*/
static int tas256x_dac_mute_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	 struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	pValue->value.integer.value[0] = p_tas256x->dac_mute;

	dev_dbg(plat_data->dev, "%s = %ld\n",
		__func__, pValue->value.integer.value[0]);

	return 0;
}

static int tas256x_dac_mute_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
	int n_result = 0;
	enum channel chn = channel_left;
	int mute = pValue->value.integer.value[0];
	int i = 0, chnTemp = 0;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;

	dev_dbg(plat_data->dev, "%s, %d\n", __func__, mute);
	mutex_lock(&p_tas256x->codec_lock);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->spk_control == 1)
			chnTemp |= 1<<i;
	}
	chn = (chnTemp == 0)?chn:(enum channel)chnTemp;

	if (mute) {
		n_result = tas256x_set_power_mute(p_tas256x, chn);
	} else {
		msleep(50);
		n_result = tas256x_set_power_up(p_tas256x, chn);
	}

	p_tas256x->dac_mute = mute;
	mutex_unlock(&p_tas256x->codec_lock);

	return n_result;
}

/*ICN Disable*/
static const char * const icn_sw_text[] = {"Enable", "Disable"};
static const struct soc_enum icn_sw_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(icn_sw_text),
	icn_sw_text),
};
static int tas256x_get_icn_switch(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *p_u_control)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;

	dev_info(plat_data->dev, "%s, icn_sw = %ld\n",
			__func__, p_u_control->value.integer.value[0]);
	p_u_control->value.integer.value[0] = p_tas256x->icn_sw;
	return 0;
}
static int tas256x_set_icn_switch(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *p_u_control)
{
	int ret  = 0;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif

	p_tas256x->icn_sw = p_u_control->value.integer.value[0];
	ret = tas256x_icn_disable(p_tas256x, p_tas256x->icn_sw, channel_both);

	return ret;
}

static const struct snd_kcontrol_new tas256x_snd_controls[] = {
	SOC_SINGLE_EXT("SmartPA System Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_system_mute_ctrl_get, tas256x_system_mute_ctrl_put),
	SOC_SINGLE_EXT("SmartPA Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_mute_ctrl_get, tas256x_mute_ctrl_put),
	SOC_SINGLE_EXT("TAS256X DAC Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_dac_mute_ctrl_get, tas256x_dac_mute_ctrl_put),
	SOC_ENUM_EXT("TAS256X ICN Switch", icn_sw_enum[0],
		tas256x_get_icn_switch,
		tas256x_set_icn_switch),
};

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static struct snd_soc_component_driver soc_codec_driver_tas256x = {
	.probe			= tas256x_codec_probe,
	.remove			= tas256x_codec_remove,
	.read			= tas256x_codec_read,
	.write			= tas256x_codec_write,
#ifdef CODEC_PM
	.suspend		= tas256x_codec_suspend,
	.resume			= tas256x_codec_resume,
#endif
	.controls		= tas256x_snd_controls,
	.num_controls		= ARRAY_SIZE(tas256x_snd_controls),
	.dapm_widgets		= tas256x_dapm_widgets_mono,
	.num_dapm_widgets	= ARRAY_SIZE(tas256x_dapm_widgets_mono),
	.dapm_routes		= tas256x_audio_map_mono,
	.num_dapm_routes	= ARRAY_SIZE(tas256x_audio_map_mono),
};
#else
static struct snd_soc_codec_driver soc_codec_driver_tas256x = {
	.probe			= tas256x_codec_probe,
	.remove			= tas256x_codec_remove,
	.read			= tas256x_codec_read,
	.write			= tas256x_codec_write,
#ifdef CODEC_PM
	.suspend		= tas256x_codec_suspend,
	.resume			= tas256x_codec_resume,
#endif
	.component_driver = {
		.controls		= tas256x_snd_controls,
		.num_controls		= ARRAY_SIZE(tas256x_snd_controls),
		.dapm_widgets		= tas256x_dapm_widgets_mono,
		.num_dapm_widgets	= ARRAY_SIZE(tas256x_dapm_widgets_mono),
		.dapm_routes		= tas256x_audio_map_mono,
		.num_dapm_routes	= ARRAY_SIZE(tas256x_audio_map_mono),
	},
};
#endif

int tas256x_register_codec(struct tas256x_priv *p_tas256x)
{
	int n_result = 0;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	dev_info(plat_data->dev, "%s, enter\n", __func__);

	if (p_tas256x->mn_channels == 2) {
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
		soc_codec_driver_tas256x.dapm_widgets =
			tas256x_dapm_widgets_stereo;
		soc_codec_driver_tas256x.num_dapm_widgets =
			ARRAY_SIZE(tas256x_dapm_widgets_stereo);
		soc_codec_driver_tas256x.dapm_routes =
			tas256x_audio_map_stereo;
		soc_codec_driver_tas256x.num_dapm_routes =
			ARRAY_SIZE(tas256x_audio_map_stereo);
		n_result = devm_snd_soc_register_component(plat_data->dev,
			&soc_codec_driver_tas256x,
			tas256x_dai_driver, ARRAY_SIZE(tas256x_dai_driver));
#else
		soc_codec_driver_tas256x.component_driver.dapm_widgets =
			tas256x_dapm_widgets_stereo;
		soc_codec_driver_tas256x.component_driver.num_dapm_widgets =
			ARRAY_SIZE(tas256x_dapm_widgets_stereo);
		soc_codec_driver_tas256x.component_driver.dapm_routes =
			tas256x_audio_map_stereo;
		soc_codec_driver_tas256x.component_driver.num_dapm_routes =
			ARRAY_SIZE(tas256x_audio_map_stereo);
		n_result = snd_soc_register_codec(plat_data->dev,
			&soc_codec_driver_tas256x,
			tas256x_dai_driver, ARRAY_SIZE(tas256x_dai_driver));
#endif
	} else {
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
		n_result = devm_snd_soc_register_component(plat_data->dev,
			&soc_codec_driver_tas256x,
			tas256x_dai_driver, ARRAY_SIZE(tas256x_dai_driver));
#else
		n_result = snd_soc_register_codec(plat_data->dev,
			&soc_codec_driver_tas256x,
			tas256x_dai_driver, ARRAY_SIZE(tas256x_dai_driver));
#endif
	}
	return n_result;
}

int tas256x_deregister_codec(struct tas256x_priv *p_tas256x)
{
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	snd_soc_unregister_component(plat_data->dev);
#else
	snd_soc_unregister_codec(plat_data->dev);
#endif
	return 0;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS256X ALSA SOC Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif /* CONFIG_TAS256X_CODEC */
