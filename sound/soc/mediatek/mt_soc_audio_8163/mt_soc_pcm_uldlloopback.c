/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */



/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_pcm_common.h"

/*
 *    function implementation
 */

enum {
	AP_LOOPBACK_NONE = 0,
	AP_LOOPBACK_AMIC_TO_SPK,
	AP_LOOPBACK_AMIC_TO_HP,
	AP_LOOPBACK_HEADSET_MIC_TO_SPK,
	AP_LOOPBACK_HEADSET_MIC_TO_HP,
};

struct mt_pcm_uldlloopback_priv {
	uint32_t ap_loopback_type;
};

static int ap_loopback_get(struct snd_kcontrol *kcontrol,
	 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	/* struct mt_pcm_uldlloopback_priv *priv = */
	/* snd_soc_component_get_drvdata(component); */
	struct mt_pcm_uldlloopback_priv *priv =
		 dev_get_drvdata(component->dev);

	ucontrol->value.integer.value[0] = priv->ap_loopback_type;
	return 0;
}

static int ap_loopback_set(struct snd_kcontrol *kcontrol,
	 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	/* struct mt_pcm_uldlloopback_priv *priv = */
	/* snd_soc_component_get_drvdata(component); */
	struct mt_pcm_uldlloopback_priv *priv =
		 dev_get_drvdata(component->dev);
	uint32_t sample_rate = 48000;
	long set_value = ucontrol->value.integer.value[0];

	if (priv->ap_loopback_type == set_value) {
		pr_debug("%s dummy operation for %u",
			 __func__, priv->ap_loopback_type);
		return 0;
	}

	if (priv->ap_loopback_type != AP_LOOPBACK_NONE) {

		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, false);
		if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC)
			== false)
			SetI2SAdcEnable(false);

		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, false);
		if (GetI2SDacEnable() == false)
			SetI2SDacEnable(false);

		SetConnection(Soc_Aud_InterCon_DisConnect,
			 Soc_Aud_InterConnectionInput_I03,
			Soc_Aud_InterConnectionOutput_O03);
		SetConnection(Soc_Aud_InterCon_DisConnect,
			 Soc_Aud_InterConnectionInput_I04,
			Soc_Aud_InterConnectionOutput_O04);

		EnableAfe(false);

		AudDrv_Clk_Off();
		AudDrv_ADC_Clk_Off();
		AudDrv_ANA_Clk_Off();
	}

	if (set_value == AP_LOOPBACK_AMIC_TO_SPK ||
	    set_value == AP_LOOPBACK_AMIC_TO_HP ||
	    set_value == AP_LOOPBACK_HEADSET_MIC_TO_SPK ||
	    set_value == AP_LOOPBACK_HEADSET_MIC_TO_HP) {

		struct AudioDigtalI2S AudI2SConfig;

		AudI2SConfig.mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
		AudI2SConfig.mBuffer_Update_word = 8;
		AudI2SConfig.mFpga_bit_test = 0;
		AudI2SConfig.mFpga_bit = 0;
		AudI2SConfig.mloopback = 0;
		AudI2SConfig.mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
		AudI2SConfig.mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
		AudI2SConfig.mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_32BITS;
		AudI2SConfig.mI2S_SAMPLERATE = sample_rate;

		AudDrv_ANA_Clk_On();
		AudDrv_Clk_On();
		AudDrv_ADC_Clk_On();

		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1,
			 AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL2,
			 AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
			 Soc_Aud_InterConnectionOutput_O03);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
			 Soc_Aud_InterConnectionOutput_O04);

		SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S,  sample_rate);

		/* configure downlink */
		if (GetMemoryPathEnable(
			Soc_Aud_Digital_Block_I2S_OUT_DAC) == false) {
			SetMemoryPathEnable(
			Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
			SetI2SDacOut(sample_rate, false,
				OUTPUT_DATA_FORMAT_24BIT);
			SetI2SDacEnable(true);
		} else {
			SetMemoryPathEnable(
			Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
		}


		/* configure uplink */
		if (GetMemoryPathEnable(
			Soc_Aud_Digital_Block_I2S_IN_ADC) == false) {
			SetI2SAdcIn(&AudI2SConfig);
			SetMemoryPathEnable(
			Soc_Aud_Digital_Block_I2S_IN_ADC, true);
			SetI2SAdcEnable(true);
		} else
			SetMemoryPathEnable(
				Soc_Aud_Digital_Block_I2S_IN_ADC,
				 true);

		SetConnection(Soc_Aud_InterCon_Connection,
			 Soc_Aud_InterConnectionInput_I03,
			Soc_Aud_InterConnectionOutput_O03);
		SetConnection(Soc_Aud_InterCon_Connection,
			 Soc_Aud_InterConnectionInput_I04,
			Soc_Aud_InterConnectionOutput_O04);

		EnableAfe(true);
	}

	priv->ap_loopback_type = ucontrol->value.integer.value[0];

	return 0;
}

static const char *const ap_loopback_function[] = {
	"AP_LOOPBACK_NONE",
	"AP_LOOPBACK_AMIC_TO_SPK",
	"AP_LOOPBACK_AMIC_TO_HP",
	"AP_LOOPBACK_HEADSET_MIC_TO_SPK",
	"AP_LOOPBACK_HEADSET_MIC_TO_HP",
};

static const struct soc_enum mt_pcm_loopback_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ap_loopback_function),
		 ap_loopback_function),
};

static const struct snd_kcontrol_new mt_pcm_loopback_controls[] = {
	SOC_ENUM_EXT("AP_Loopback_Select",
		 mt_pcm_loopback_control_enum[0], ap_loopback_get,
		     ap_loopback_set),
};

static int mtk_uldlloopback_probe(struct platform_device *pdev);
static int mtk_uldlloopbackpcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_uldlloopbackpcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_uldlloopback_probe(struct snd_soc_platform *platform);

static struct snd_pcm_hardware mtk_uldlloopback_hardware = {

	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SND_SOC_STD_MT_FMTS,
	.rates = SOC_HIGH_USE_RATE,
	.rate_min = SOC_NORMAL_USE_RATE_MIN,
	.rate_max = SOC_NORMAL_USE_RATE_MAX,
	.channels_min = SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max = SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = Dl1_MAX_BUFFER_SIZE,
	.period_bytes_max = MAX_PERIOD_SIZE,
	.periods_min = MIN_PERIOD_SIZE,
	.periods_max = MAX_PERIOD_SIZE,
	.fifo_size = 0,
};

static struct snd_soc_pcm_runtime *pruntimepcm;

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {

	.count = ARRAY_SIZE(soc_high_supported_sample_rates),
	.list = soc_high_supported_sample_rates,
	.mask = 0,
};

static int mtk_uldlloopback_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err = 0;
	int ret = 0;

	pr_debug("%s\n", __func__);
	AudDrv_ANA_Clk_On();
	AudDrv_Clk_On();
	AudDrv_ADC_Clk_On();
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_err("%s with SNDRV_PCM_STREAM_CAPTURE\n", __func__);
		runtime->rate = 48000;
		return 0;
	}

	runtime->hw = mtk_uldlloopback_hardware;
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_uldlloopback_hardware,
	       sizeof(struct snd_pcm_hardware));

	ret = snd_pcm_hw_constraint_list(runtime, 0,
		 SNDRV_PCM_HW_PARAM_RATE,
	 &constraints_sample_rates);
	ret = snd_pcm_hw_constraint_integer(runtime,
		 SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0)
		pr_err("snd_pcm_hw_constraint_integer failed\n");

	/* print for hw pcm information */
	pr_debug("mtk_uldlloopback_open runtime->rate = %d channels = %d\n",
		runtime->rate, runtime->channels);

	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
	runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pr_debug("SNDRV_PCM_STREAM_PLAYBACK mtkalsa_voice_constraints\n");

	if (err < 0) {
		pr_err("mtk_uldlloopbackpcm_close\n");
		mtk_uldlloopbackpcm_close(substream);
		return err;
	}
	pr_debug("mtk_uldlloopback_open return\n");
	return 0;
}

static int mtk_uldlloopbackpcm_close(struct snd_pcm_substream *substream)
{
	pr_debug("%s\n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_err("%s with SNDRV_PCM_STREAM_CAPTURE\n", __func__);
		return 0;
	}

	/* interconnection setting */
	SetConnection(Soc_Aud_InterCon_DisConnect,
		 Soc_Aud_InterConnectionInput_I03,
		      Soc_Aud_InterConnectionOutput_O00);
	SetConnection(Soc_Aud_InterCon_DisConnect,
		 Soc_Aud_InterConnectionInput_I04,
		      Soc_Aud_InterConnectionOutput_O01);
	SetConnection(Soc_Aud_InterCon_DisConnect,
		 Soc_Aud_InterConnectionInput_I03,
		      Soc_Aud_InterConnectionOutput_O03);
	SetConnection(Soc_Aud_InterCon_DisConnect,
		 Soc_Aud_InterConnectionInput_I04,
		      Soc_Aud_InterConnectionOutput_O04);


	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, false);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC) == false)
		SetI2SAdcEnable(false);

	/* stop DAC output */
	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, false);
	if (GetI2SDacEnable() == false)
		SetI2SDacEnable(false);

	/* stop I2S */
	Afe_Set_Reg(AFE_I2S_CON3, 0x0, 0x1);
	Afe_Set_Reg(AFE_I2S_CON, 0x0, 0x1);
	Afe_Set_Reg(AFE_I2S_CON1, 0x0, 0x1);
	Afe_Set_Reg(AFE_I2S_CON2, 0x0, 0x1);

	EnableAfe(false);

	AudDrv_Clk_Off();
	AudDrv_ADC_Clk_Off();
	AudDrv_ANA_Clk_Off();
	return 0;
}

static int mtk_uldlloopbackpcm_trigger(struct snd_pcm_substream *substream,
	 int cmd)
{
	pr_debug("%s cmd = %d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	}
	return -EINVAL;
}

static int mtk_uldlloopback_pcm_copy(struct snd_pcm_substream *substream,
				     int channel, snd_pcm_uframes_t pos,
				     void __user *dst, snd_pcm_uframes_t count)
{
	count = count << 2;
	return 0;
}

static int mtk_uldlloopback_silence(struct snd_pcm_substream *substream,
	int channel, snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	pr_debug("dummy_pcm_silence\n");
	return 0;		/* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_uldlloopback_page(struct snd_pcm_substream *substream,
	 unsigned long offset)
{
	pr_debug("dummy_pcm_page\n");
	return virt_to_page(dummy_page[substream->stream]);
}

static struct AudioDigtalI2S mAudioDigitalI2S;
static void ConfigAdcI2S(struct snd_pcm_substream *substream)
{
	mAudioDigitalI2S.mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
	mAudioDigitalI2S.mBuffer_Update_word = 8;
	mAudioDigitalI2S.mFpga_bit_test = 0;
	mAudioDigitalI2S.mFpga_bit = 0;
	mAudioDigitalI2S.mloopback = 0;
	mAudioDigitalI2S.mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
	mAudioDigitalI2S.mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
	mAudioDigitalI2S.mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_16BITS;
	mAudioDigitalI2S.mI2S_SAMPLERATE = (substream->runtime->rate);
}

static int mtk_uldlloopback_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	/* uint32 eSamplingRate = SampleRateTransform(runtime->rate); */
	/* uint32 dVoiceModeSelect = 0; */
	/* uint32 Audio_I2S_Dac = 0; */
	uint32 u32AudioI2S = 0;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_err("%s with SNDRV_PCM_STREAM_CAPTURE\n", __func__);
		return 0;
	}

	pr_debug("%s rate = %d\n", __func__, runtime->rate);

	Afe_Set_Reg(AFE_TOP_CON0, 0x00000000, 0xffffffff);
	if (runtime->format == SNDRV_PCM_FORMAT_S32_LE
	    || runtime->format == SNDRV_PCM_FORMAT_U32_LE) {
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1,
			AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL2,
			AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
					  Soc_Aud_InterConnectionOutput_O03);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
					  Soc_Aud_InterConnectionOutput_O04);
	} else {
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1,
			 AFE_WLEN_16_BIT);
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL2,
			 AFE_WLEN_16_BIT);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
					  Soc_Aud_InterConnectionOutput_O03);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
					  Soc_Aud_InterConnectionOutput_O04);
	}

	/* interconnection setting */
	SetConnection(Soc_Aud_InterCon_Connection,
		 Soc_Aud_InterConnectionInput_I03,
		      Soc_Aud_InterConnectionOutput_O00);
	SetConnection(Soc_Aud_InterCon_Connection,
		 Soc_Aud_InterConnectionInput_I04,
		      Soc_Aud_InterConnectionOutput_O01);
	SetConnection(Soc_Aud_InterCon_Connection,
		 Soc_Aud_InterConnectionInput_I03,
		      Soc_Aud_InterConnectionOutput_O03);
	SetConnection(Soc_Aud_InterCon_Connection,
		 Soc_Aud_InterConnectionInput_I04,
		      Soc_Aud_InterConnectionOutput_O04);

	Afe_Set_Reg(AFE_ADDA_TOP_CON0, 0, 0x1);	/* Using Internal ADC */

	u32AudioI2S |= Soc_Aud_LOW_JITTER_CLOCK << 12;	/* Low jitter mode */
	u32AudioI2S |= SampleRateTransform(runtime->rate) << 8;
	u32AudioI2S |= Soc_Aud_I2S_FORMAT_I2S << 3;	/* us3 I2s format */
	u32AudioI2S |= Soc_Aud_I2S_WLEN_WLEN_32BITS << 1;	/* 32 BITS */

	pr_debug("u32AudioI2S= 0x%x\n", u32AudioI2S);
	Afe_Set_Reg(AFE_I2S_CON3, u32AudioI2S | 1, AFE_MASK_ALL);

	/* start I2S DAC out */
	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC) == false) {
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
		SetI2SDacOut(substream->runtime->rate,
			 false, Soc_Aud_I2S_WLEN_WLEN_32BITS);
		SetI2SDacEnable(true);
	} else
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC) == false) {
		ConfigAdcI2S(substream);
		SetI2SAdcIn(&mAudioDigitalI2S);
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, true);
		SetI2SAdcEnable(true);
	} else
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, true);

	EnableAfe(true);

	return 0;
}

static int mtk_uldlloopback_pcm_hw_params(struct snd_pcm_substream *substream,
					  struct snd_pcm_hw_params *hw_params)
{
	int ret = 0;

	pr_debug("mtk_uldlloopback_pcm_hw_params\n");
	return ret;
}

static int mtk_uldlloopback_pcm_hw_free(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_uldlloopback_pcm_hw_free\n");
	return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_ops mtk_afe_ops = {

	.open = mtk_uldlloopback_open,
	.close = mtk_uldlloopbackpcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mtk_uldlloopback_pcm_hw_params,
	.hw_free = mtk_uldlloopback_pcm_hw_free,
	.prepare = mtk_uldlloopback_pcm_prepare,
	.trigger = mtk_uldlloopbackpcm_trigger,
	.copy = mtk_uldlloopback_pcm_copy,
	.silence = mtk_uldlloopback_silence,
	.page = mtk_uldlloopback_page,
};

static struct snd_soc_platform_driver mtk_soc_dummy_platform = {

	.ops = &mtk_afe_ops,
	.pcm_new = mtk_asoc_uldlloopbackpcm_new,
	.probe = mtk_afe_uldlloopback_probe,
};

static int mtk_uldlloopback_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt_pcm_uldlloopback_priv *priv;

	pr_debug("mtk_uldlloopback_probe\n");

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_ULDLLOOPBACK_PCM);

	priv = devm_kzalloc(dev, sizeof(struct mt_pcm_uldlloopback_priv),
		 GFP_KERNEL);
	if (unlikely(!priv)) {
		pr_err("%s failed to allocate private data\n", __func__);
		return -ENOMEM;
	}

	dev_set_drvdata(dev, priv);

	pr_debug("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev, &mtk_soc_dummy_platform);
}

static int mtk_asoc_uldlloopbackpcm_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;

	pruntimepcm = rtd;
	pr_debug("%s\n", __func__);
	return ret;
}


static int mtk_afe_uldlloopback_probe(struct snd_soc_platform *platform)
{
	pr_debug("mtk_afe_uldlloopback_probe\n");

	snd_soc_add_platform_controls(platform, mt_pcm_loopback_controls,
				      ARRAY_SIZE(mt_pcm_loopback_controls));
	return 0;
}


static int mtk_afe_uldlloopback_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_uldlloopback_of_ids[] = {

	{.compatible = "mediatek,mt8163-soc-pcm-uldlloopback",},
	{}
};
#endif

static struct platform_driver mtk_afe_uldllopback_driver = {

	.driver = {
		   .name = MT_SOC_ULDLLOOPBACK_PCM,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = mt_soc_pcm_uldlloopback_of_ids,
#endif
		   },
	.probe = mtk_uldlloopback_probe,
	.remove = mtk_afe_uldlloopback_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkafe_uldlloopback_dev;
#endif

static int __init mtk_soc_uldlloopback_platform_init(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);
#ifndef CONFIG_OF
	soc_mtkafe_uldlloopback_dev =
		 platform_device_alloc(MT_SOC_ULDLLOOPBACK_PCM, -1);

	if (!soc_mtkafe_uldlloopback_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_mtkafe_uldlloopback_dev);
	if (ret != 0) {
		platform_device_put(soc_mtkafe_uldlloopback_dev);
		return ret;
	}
#endif
	ret = platform_driver_register(&mtk_afe_uldllopback_driver);

	return ret;

}
module_init(mtk_soc_uldlloopback_platform_init);

static void __exit mtk_soc_uldlloopback_platform_exit(void)
{

	pr_debug("%s\n", __func__);
	platform_driver_unregister(&mtk_afe_uldllopback_driver);
}
module_exit(mtk_soc_uldlloopback_platform_exit);

MODULE_DESCRIPTION("loopback module platform driver");
MODULE_LICENSE("GPL");
