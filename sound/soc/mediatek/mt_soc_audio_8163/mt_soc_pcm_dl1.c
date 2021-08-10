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

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_digital_type.h"
#include "mt_soc_pcm_common.h"
#include "AudDrv_Gpio.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
//#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <linux/reboot.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#ifdef CONFIG_MTK_LEGACY
static unsigned int pin_extspkamp, pin_extspkamp_2,
	 pin_vowclk, pin_audclk, pin_audmiso,
	pin_audmosi, pin_i2s1clk, pin_i2s1dat, pin_i2s1mclk,
	 pin_i2s1ws, pin_rcvspkswitch;

static unsigned int pin_mode_audclk, pin_mode_audmosi,
	 pin_mode_audmiso, pin_mode_vowclk,
	pin_mode_extspkamp, pin_mode_extspkamp_2,
		 pin_mode_i2s1clk, pin_mode_i2s1dat, pin_mode_i2s1mclk,
	pin_mode_i2s1ws, pin_mode_rcvspkswitch;

static unsigned int if_config1, if_config2, if_config3,
	 if_config4, if_config5, if_config6,
	if_config7, if_config8, if_config9, if_config10, if_config11;
#endif
#endif

static struct AFE_MEM_CONTROL_T *pMemControl;
static int mPlaybackSramState;
static struct snd_dma_buffer *Dl1_Playback_dma_buf;

static DEFINE_SPINLOCK(auddrv_DLCtl_lock);

static struct device *mDev;
static int afe_emergency_stop;
/*
 *    function implementation
 */

/* void StartAudioPcmHardware(void); */
/* void StopAudioPcmHardware(void); */
static int mtk_soc_dl1_probe(struct platform_device *pdev);
static int mtk_soc_pcm_dl1_close(struct snd_pcm_substream *substream);
static int mtk_asoc_pcm_dl1_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_asoc_dl1_probe(struct snd_soc_platform *platform);

static bool mPrepareDone;

#define USE_RATE (SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000)
#define USE_RATE_MIN        8000
#define USE_RATE_MAX        192000
#define USE_CHANNELS_MIN     1
#define USE_CHANNELS_MAX    2
#define USE_PERIODS_MIN     512
#define USE_PERIODS_MAX     8192

static struct snd_pcm_hardware mtk_pcm_dl1_hardware = {

	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SND_SOC_ADV_MT_FMTS,
	.rates = SOC_HIGH_USE_RATE,
	.rate_min = SOC_HIGH_USE_RATE_MIN,
	.rate_max = SOC_HIGH_USE_RATE_MAX,
	.channels_min = SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max = SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = Dl1_MAX_BUFFER_SIZE,
	.period_bytes_max = Dl1_MAX_PERIOD_SIZE,
	.periods_min = SOC_NORMAL_USE_PERIODS_MIN,
	.periods_max = SOC_NORMAL_USE_PERIODS_MAX,
	.fifo_size = 0,
};

static int mtk_pcm_dl1_stop(struct snd_pcm_substream *substream)
{
	pr_debug("%s\n", __func__);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, false);
	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, false);

	/* here start digital part */
	SetConnection(Soc_Aud_InterCon_DisConnect,
		 Soc_Aud_InterConnectionInput_I05,
		      Soc_Aud_InterConnectionOutput_O03);
	SetConnection(Soc_Aud_InterCon_DisConnect,
		 Soc_Aud_InterConnectionInput_I06,
		      Soc_Aud_InterConnectionOutput_O04);

	ClearMemBlock(Soc_Aud_Digital_Block_MEM_DL1);
	return 0;
}

static snd_pcm_uframes_t mtk_pcm_pointer(struct snd_pcm_substream *substream)
{
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	kal_uint32 Frameidx = 0;
	kal_int32 Afe_consumed_bytes = 0;
	struct AFE_BLOCK_T *Afe_Block = &pMemControl->rBlock;
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	pr_debug("%s Afe_Block->u4DMAReadIdx = 0x%x\n",
		 __func__, Afe_Block->u4DMAReadIdx);

	Auddrv_Dl1_Spinlock_lock();

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1) == true) {
		HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL1_CUR);
		if (HW_Cur_ReadIdx == 0) {
			pr_debug("[%s] HW_Cur_ReadIdx == 0\n", __func__);
			HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
		}

		HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);

		if (HW_memory_index >= Afe_Block->u4DMAReadIdx)
			Afe_consumed_bytes =
				HW_memory_index - Afe_Block->u4DMAReadIdx;
		else {
			Afe_consumed_bytes =
			    Afe_Block->u4BufferSize +
			     HW_memory_index - Afe_Block->u4DMAReadIdx;
		}

		Afe_consumed_bytes = Align64ByteSize(Afe_consumed_bytes);

		Afe_Block->u4DataRemained -= Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx += Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;

		pr_debug
			("[%s] HW_Cur_ReadIdx = 0x%x  0x%x  0x%x\n",
			__func__, HW_Cur_ReadIdx,
			HW_memory_index, Afe_consumed_bytes);

		Auddrv_Dl1_Spinlock_unlock();

		return audio_bytes_to_frame(substream, Afe_Block->u4DMAReadIdx);
	}

	Frameidx = audio_bytes_to_frame(substream, Afe_Block->u4DMAReadIdx);
	Auddrv_Dl1_Spinlock_unlock();
	return Frameidx;
}

static void SetDL1Buffer(struct snd_pcm_substream *substream,
	 struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct AFE_BLOCK_T *pblock = &pMemControl->rBlock;

	pblock->pucPhysBufAddr = runtime->dma_addr;
	pblock->pucVirtBufAddr = runtime->dma_area;
	pblock->u4BufferSize = runtime->dma_bytes;
	pblock->u4SampleNumMask = 0x001f;	/* 32 byte align */
	pblock->u4WriteIdx = 0;
	pblock->u4DMAReadIdx = 0;
	pblock->u4DataRemained = 0;
	pblock->u4fsyncflag = false;
	pblock->uResetFlag = true;

	Afe_Set_Reg(AFE_DL1_BASE, pblock->pucPhysBufAddr, 0xffffffff);
	Afe_Set_Reg(AFE_DL1_END, pblock->pucPhysBufAddr +
		 (pblock->u4BufferSize - 1), 0xffffffff);
	memset((void *)pblock->pucVirtBufAddr, 0, pblock->u4BufferSize);

}

static int mtk_pcm_dl1_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *hw_params)
{
	/* struct snd_dma_buffer *dma_buf = &substream->dma_buffer; */
	int ret = 0;

	pr_debug("mtk_pcm_dl1_params\n");

	/* runtime->dma_bytes has to be set manually to allow mmap */
	substream->runtime->dma_bytes = params_buffer_bytes(hw_params);

	if (mPlaybackSramState == SRAM_STATE_PLAYBACKFULL) {
		/* substream->runtime->dma_bytes = AFE_INTERNAL_SRAM_SIZE; */
		substream->runtime->dma_area =
			 (unsigned char *)Get_Afe_SramBase_Pointer();
		substream->runtime->dma_addr = AFE_INTERNAL_SRAM_PHY_BASE;
		AudDrv_Allocate_DL1_Buffer(mDev, substream->runtime->dma_bytes);
	} else {
		substream->runtime->dma_bytes =
			 params_buffer_bytes(hw_params);
		substream->runtime->dma_area =
			 Dl1_Playback_dma_buf->area;
		substream->runtime->dma_addr =
			 Dl1_Playback_dma_buf->addr;
		SetDL1Buffer(substream, hw_params);
	}

	pr_debug
	("dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	substream->runtime->dma_bytes,
	substream->runtime->dma_area,
	(long)substream->runtime->dma_addr);
	return ret;
}

static int mtk_pcm_dl1_hw_free(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_pcm_dl1_hw_free\n");
	return 0;
}


static struct snd_pcm_hw_constraint_list constraints_sample_rates = {

	.count = ARRAY_SIZE(soc_high_supported_sample_rates),
	.list = soc_high_supported_sample_rates,
	.mask = 0,
};

static int mtk_pcm_dl1_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_debug("mtk_pcm_dl1_open\n");

	AfeControlSramLock();
	if (GetSramState() == SRAM_STATE_FREE) {
		mtk_pcm_dl1_hardware.buffer_bytes_max =
			 GetPLaybackSramFullSize();
		mPlaybackSramState = SRAM_STATE_PLAYBACKFULL;
		SetSramState(mPlaybackSramState);
	} else {
		mtk_pcm_dl1_hardware.buffer_bytes_max =
			 GetPLaybackDramSize();
		mPlaybackSramState = SRAM_STATE_PLAYBACKDRAM;
	}
	AfeControlSramUnLock();

	if (mPlaybackSramState == SRAM_STATE_PLAYBACKDRAM)
		AudDrv_Emi_Clk_On();

	pr_debug
	("buffer_bytes_max =%zu mPlaybackSramState = %d\n",
	mtk_pcm_dl1_hardware.buffer_bytes_max, mPlaybackSramState);
	runtime->hw = mtk_pcm_dl1_hardware;

	AudDrv_ANA_Clk_On();
	AudDrv_Clk_On();
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_pcm_dl1_hardware,
	       sizeof(struct snd_pcm_hardware));
	pMemControl = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_DL1);

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_sample_rates);

	if (ret < 0)
		pr_warn("snd_pcm_hw_constraint_integer failed\n");

	if (ret < 0) {
		pr_err("ret < 0 mtk_soc_pcm_dl1_close\n");
		mtk_soc_pcm_dl1_close(substream);
		return ret;
	}

	/* pr_debug("mtk_pcm_dl1_open return\n"); */
	return 0;
}

static int mtk_soc_pcm_dl1_close(struct snd_pcm_substream *substream)
{
	pr_debug("%s\n", __func__);
	if (mPrepareDone == true) {
		/* stop DAC output */
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, false);

		if (GetI2SDacEnable() == false)
			SetI2SDacEnable(false);

		RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_DL1, substream);

		EnableAfe(false);
		mPrepareDone = false;
	}

	if (mPlaybackSramState == SRAM_STATE_PLAYBACKDRAM)
		AudDrv_Emi_Clk_Off();

	AfeControlSramLock();
	ClearSramState(mPlaybackSramState);
	mPlaybackSramState = GetSramState();
	AfeControlSramUnLock();
	AudDrv_Clk_Off();
	AudDrv_ANA_Clk_Off();
	return 0;
}

static int mtk_pcm_prepare(struct snd_pcm_substream *substream)
{
	bool mI2SWLen;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (mPrepareDone == false) {
		pr_debug("%s format = %d  %d  %d\n",
		__func__,
		runtime->format, SNDRV_PCM_FORMAT_S32_LE,
		SNDRV_PCM_FORMAT_U32_LE);
		SetMemifSubStream(Soc_Aud_Digital_Block_MEM_DL1, substream);

		if (runtime->format == SNDRV_PCM_FORMAT_S32_LE
		    || runtime->format == SNDRV_PCM_FORMAT_U32_LE) {
			SetMemIfFetchFormatPerSample(
				Soc_Aud_Digital_Block_MEM_DL1,
				AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
			SetMemIfFetchFormatPerSample(
				Soc_Aud_Digital_Block_MEM_DL2,
				AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
			SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
				Soc_Aud_InterConnectionOutput_O03);
			SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
				Soc_Aud_InterConnectionOutput_O04);
			mI2SWLen = Soc_Aud_I2S_WLEN_WLEN_32BITS;
		} else {
			SetMemIfFetchFormatPerSample(
				Soc_Aud_Digital_Block_MEM_DL1,
				AFE_WLEN_16_BIT);
			SetMemIfFetchFormatPerSample(
				Soc_Aud_Digital_Block_MEM_DL2,
				AFE_WLEN_16_BIT);
			SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
				Soc_Aud_InterConnectionOutput_O03);
			SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
				Soc_Aud_InterConnectionOutput_O04);
			mI2SWLen = Soc_Aud_I2S_WLEN_WLEN_16BITS;
		}

		SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, runtime->rate);

		/* start I2S DAC out */
		if (GetMemoryPathEnable(
			Soc_Aud_Digital_Block_I2S_OUT_DAC) == false) {
			SetMemoryPathEnable(
				Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
			SetI2SDacOut(substream->runtime->rate,
				 false, mI2SWLen);
			SetI2SDacEnable(true);
		} else
			SetMemoryPathEnable(
				Soc_Aud_Digital_Block_I2S_OUT_DAC, true);

		/* here to set interrupt_distributor */
		SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE,
			 runtime->period_size);
		SetIrqMcuSampleRate(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE,
			 runtime->rate);

		EnableAfe(true);
		mPrepareDone = true;
	}
	return 0;
}


static int mtk_pcm_dl1_start(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_debug("%s\n", __func__);
	/* here start digital part */

	if (unlikely(afe_emergency_stop))
		return -EINVAL;

	SetConnection(Soc_Aud_InterCon_Connection,
		 Soc_Aud_InterConnectionInput_I05,
		      Soc_Aud_InterConnectionOutput_O03);
	SetConnection(Soc_Aud_InterCon_Connection,
		 Soc_Aud_InterConnectionInput_I06,
		      Soc_Aud_InterConnectionOutput_O04);

	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, true);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_DL1, runtime->rate);
	SetChannels(Soc_Aud_Digital_Block_MEM_DL1, runtime->channels);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, true);

	EnableAfe(true);

	return 0;
}

static int mtk_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	pr_debug("mtk_pcm_trigger cmd = %d\n", cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_pcm_dl1_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_pcm_dl1_stop(substream);
	}
	return -EINVAL;
}

static int mtk_pcm_copy(struct snd_pcm_substream *substream,
			int channel, snd_pcm_uframes_t pos,
			void __user *dst, snd_pcm_uframes_t count)
{
	struct AFE_BLOCK_T *Afe_Block = NULL;
	int copy_size = 0, Afe_WriteIdx_tmp;
	unsigned long flags;
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	char *data_w_ptr = (char *)dst;

	pr_debug("mtk_pcm_copy pos = %lu count = %lu\n",
	pos, count);
	/* get total bytes to copy */
	count = audio_frame_to_bytes(substream, count);

	/* check which memif nned to be write */
	Afe_Block = &pMemControl->rBlock;

	pr_debug("%s, wp=0x%x, rp=0x%x, remained=0x%x\n",
	__func__, Afe_Block->u4WriteIdx,
	Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);

	if (Afe_Block->u4BufferSize == 0) {
		pr_err("AudDrv_write: u4BufferSize=0 Error\n");
		return 0;
	}

	spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
	copy_size = Afe_Block->u4BufferSize - Afe_Block->u4DataRemained;
	spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);

	if (count <= copy_size) {
		if (copy_size < 0)
			copy_size = 0;
		else
			copy_size = count;
	}

	copy_size = Align64ByteSize(copy_size);
	pr_debug("copy_size=0x%x, count=0x%x\n",
	copy_size, (unsigned int)count);

	if (copy_size != 0) {
		spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
		Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
		spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);

		if (Afe_WriteIdx_tmp + copy_size < Afe_Block->u4BufferSize) {
			if (!access_ok(VERIFY_READ, data_w_ptr, copy_size)) {
				pr_debug
				("write 0 data_w_ptr=%p, size=%d\n",
				data_w_ptr, copy_size);
				pr_debug
				("write u4BufferSize=%d, u4DataRemained=%d\n",
				Afe_Block->u4BufferSize,
				Afe_Block->u4DataRemained);
			} else {
				if (copy_from_user((
					Afe_Block->pucVirtBufAddr +
					Afe_WriteIdx_tmp),
					data_w_ptr, copy_size)) {
					pr_debug("write Fail copy from user\n");
					return -1;
				}
			}

			spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
			Afe_Block->u4DataRemained += copy_size;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + copy_size;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
			data_w_ptr += copy_size;
			count -= copy_size;

			pr_debug
			("%s finish 1,size=%x,remained=%x,count %d\n",
			__func__, copy_size, Afe_Block->u4DataRemained,
			(int)count);

		} else {	/* copy twice */
			kal_uint32 size_1 = 0, size_2 = 0;

			size_1 = Align64ByteSize(
				(Afe_Block->u4BufferSize - Afe_WriteIdx_tmp));
			size_2 = Align64ByteSize((copy_size - size_1));
			pr_debug("size_1=0x%x, size_2=0x%x\n",
			size_1, size_2);

			if (!access_ok(VERIFY_READ, data_w_ptr, size_1)) {
				pr_debug("write 1w_ptr=%p, size_1=%d\n",
				data_w_ptr, size_1);
				pr_debug("write size=%d, remained=%d\n",
				Afe_Block->u4BufferSize,
				Afe_Block->u4DataRemained);
			} else {
				pr_debug
				("write addr=%p, w_ptr=%p, size_1=%x\n",
				Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp,
				data_w_ptr, size_1);

				if ((copy_from_user(
					(Afe_Block->pucVirtBufAddr +
					Afe_WriteIdx_tmp),
					data_w_ptr, (unsigned int)size_1))) {
					pr_debug("write Fail 1\n");
					return -1;
				}
			}

			spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
			Afe_Block->u4DataRemained += size_1;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_1;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
			spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);

			if (!access_ok(VERIFY_READ,
				data_w_ptr + size_1, size_2)) {
				pr_debug
				("write2 fail w_ptr=%p, size1=%d, size2=%d\n",
			     data_w_ptr, size_1, size_2);
				pr_debug
				("AudDrv_write size=%d, remained=%d\n",
				Afe_Block->u4BufferSize,
				Afe_Block->u4DataRemained);
			} else {
				pr_debug
				("addr %p, size_1=%p, size_2=%x\n",
				Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp,
				data_w_ptr + size_1,
				(unsigned int)size_2);

				if ((copy_from_user((Afe_Block->pucVirtBufAddr +
					 Afe_WriteIdx_tmp),
					(data_w_ptr + size_1), size_2))) {
					pr_debug("write Fail 2\n");
					return -1;
				}
			}
			spin_lock_irqsave(&auddrv_DLCtl_lock, flags);

			Afe_Block->u4DataRemained += size_2;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_2;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
			count -= copy_size;
			data_w_ptr += copy_size;

			pr_debug
			("%s finish 2 size:%x wp:%x rp:%x remained:%x\n",
			__func__, copy_size, Afe_Block->u4WriteIdx,
			Afe_Block->u4DMAReadIdx,
			Afe_Block->u4DataRemained);
		}
	}
	return 0;
}

static int mtk_pcm_silence(struct snd_pcm_substream *substream,
	int channel, snd_pcm_uframes_t pos,
	snd_pcm_uframes_t count)
{
	pr_debug("%s\n", __func__);
	/* do nothing */
	return 0;
}

static void *dummy_page[2];

static struct page *mtk_pcm_page(
	struct snd_pcm_substream *substream, unsigned long offset)
{
	pr_debug("%s\n", __func__);
	return virt_to_page(dummy_page[substream->stream]);
}

static struct snd_pcm_ops mtk_afe_ops = {

	.open = mtk_pcm_dl1_open,
	.close = mtk_soc_pcm_dl1_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mtk_pcm_dl1_params,
	.hw_free = mtk_pcm_dl1_hw_free,
	.prepare = mtk_pcm_prepare,
	.trigger = mtk_pcm_trigger,
	.pointer = mtk_pcm_pointer,
	.copy = mtk_pcm_copy,
	.silence = mtk_pcm_silence,
	.page = mtk_pcm_page,
};
static int mtk_afe_reboot(struct notifier_block *nb,
	unsigned long action, void *data)
{
	afe_emergency_stop = 1;
	AudDrv_Clk_On();
	mtk_pcm_dl1_stop(NULL);
	AudDrv_Clk_Off();

	return NOTIFY_DONE;
}

static struct notifier_block mtk_afe_reboot_notifier = {
	.notifier_call		= mtk_afe_reboot,
	.next			= NULL,
	.priority		= INT_MAX,
};
static struct snd_soc_platform_driver mtk_soc_platform = {

	.ops = &mtk_afe_ops,
	.pcm_new = mtk_asoc_pcm_dl1_new,
	.probe = mtk_asoc_dl1_probe,
};

#ifndef CONFIG_MTK_CLKMGR
struct platform_device *auddrv_pdev;
#endif
static int mtk_soc_dl1_probe(struct platform_device *pdev)
{
#ifndef CONFIG_OF
	int ret = 0;
#endif

	pr_debug("%s\n", __func__);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_DL1_PCM);

	pr_debug("%s: dev name %s\n",
		 __func__, dev_name(&pdev->dev));

#ifndef CONFIG_MTK_CLKMGR
	Auddrv_Clk_Init(&pdev->dev);

	pm_runtime_enable(&pdev->dev);

	auddrv_pdev = pdev;
#endif

#ifndef CONFIG_MTK_LEGACY
	AudDrv_GPIO_probe(&pdev->dev);
#endif

	InitAfeControl();

#ifndef CONFIG_OF
	ret = Register_Aud_Irq(&pdev->dev, MT8163_AFE_MCU_IRQ_LINE);
#endif
	register_reboot_notifier(&mtk_afe_reboot_notifier);
	mDev = &pdev->dev;

	return snd_soc_register_platform(&pdev->dev, &mtk_soc_platform);
}

static int mtk_asoc_pcm_dl1_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;

	pr_debug("%s\n", __func__);
	return ret;
}


static int mtk_asoc_dl1_probe(struct snd_soc_platform *platform)
{
	pr_debug("mtk_asoc_dl1_probe\n");
	/* allocate dram */
	AudDrv_Allocate_mem_Buffer(platform->dev,
		 Soc_Aud_Digital_Block_MEM_DL1,
				   Dl1_MAX_BUFFER_SIZE);
	Dl1_Playback_dma_buf =
		 Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_DL1);
	return 0;
}

static int mtk_afe_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
#ifndef CONFIG_MTK_CLKMGR
	pm_runtime_disable(&pdev->dev);
	Auddrv_Clk_Deinit(&pdev->dev);
#endif
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
/* extern void *AFE_BASE_ADDRESS; */
u32 afe_irq_number;
int AFE_BASE_PHY;

static const struct of_device_id mt_soc_pcm_dl1_of_ids[] = {

	{.compatible = "mediatek,mt8163-soc-pcm-dl1",},
	{}
};

#ifndef CONFIG_MTK_CLKMGR
int power_on_audsys(void)
{
	return pm_runtime_get_sync(&auddrv_pdev->dev);
}
EXPORT_SYMBOL(power_on_audsys);

int power_off_audsys(void)
{
	return pm_runtime_put_sync(&auddrv_pdev->dev);
}
EXPORT_SYMBOL(power_off_audsys);

#endif

static int Auddrv_Reg_map_new(void)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL,
		 NULL, "mediatek,mt8163-soc-pcm-dl1");

	if (node) {
		/* Setup IO addresses */
		AFE_BASE_ADDRESS = of_iomap(node, 0);
		pr_debug("[mt_soc_pcm_dl1] AFE_BASE_ADDRESS=0x%p\n",
			 AFE_BASE_ADDRESS);
	} else
		pr_warn("[mt_soc_pcm_dl1]NULL, can't iomapAFE_BASE!\n");

	of_property_read_u32(node, "reg", &AFE_BASE_PHY);
	pr_debug("[mt_soc_pcm_dl1] AFE_BASE_PHY=0x%x\n", AFE_BASE_PHY);

	/*get afe irq num */
	afe_irq_number = irq_of_parse_and_map(node, 0);
	pr_debug("[mt_soc_pcm_dl1] afe_irq_number=0x%x\n", afe_irq_number);
	if (!afe_irq_number) {
		pr_err("[mt_soc_pcm_dl1] get afe_irq_number failed!!!\n");
		return -1;
	}
	return 0;
}

#ifdef CONFIG_MTK_LEGACY

static int Auddrv_OF_ParseGPIO(void)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL,
		 NULL, "mediatek,mt8163-soc-pcm-dl1");
	if (node) {
		if_config1 = 1;
		if_config2 = 1;
		if_config3 = 1;
		if_config4 = 1;
		if_config5 = 1;
		if_config6 = 1;
		if_config7 = 1;
		if_config8 = 1;
		if_config9 = 1;
		if_config10 = 1;
		if_config11 = 1;

		if (of_property_read_u32_index(node,
			 "audclk-gpio", 0, &pin_audclk)) {
			if_config1 = 0;
			pr_warn("audclk-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "audclk-gpio", 1, &pin_mode_audclk)) {
			if_config1 = 0;
			pr_warn("audclk-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "audmiso-gpio", 0, &pin_audmiso)) {
			if_config2 = 0;
			pr_warn("audmiso-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "audmiso-gpio", 1, &pin_mode_audmiso)) {
			if_config2 = 0;
			pr_warn("audmiso-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "audmosi-gpio", 0, &pin_audmosi)) {
			if_config3 = 0;
			pr_warn("audmosi-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "audmosi-gpio", 1, &pin_mode_audmosi)) {
			if_config3 = 0;
			pr_warn("audmosi-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "vowclk-gpio", 0, &pin_vowclk)) {
			if_config4 = 0;
			pr_warn("vowclk-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "vowclk-gpio", 1, &pin_mode_vowclk)) {
			if_config4 = 0;
			pr_warn("vowclk-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "extspkamp-gpio", 0, &pin_extspkamp)) {
			if_config5 = 0;
			pr_warn("extspkamp-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "extspkamp-gpio", 1, &pin_mode_extspkamp)) {
			if_config5 = 0;
			pr_warn("extspkamp-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "i2s1clk-gpio", 0, &pin_i2s1clk)) {
			if_config6 = 0;
			pr_warn("i2s1clk-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "i2s1clk-gpio", 1, &pin_mode_i2s1clk)) {
			if_config6 = 0;
			pr_warn("i2s1clk-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "i2s1dat-gpio", 0, &pin_i2s1dat)) {
			if_config7 = 0;
			pr_warn("i2s1dat-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "i2s1dat-gpio", 1, &pin_mode_i2s1dat)) {
			if_config7 = 0;
			pr_warn("i2s1dat-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node, "i2s1mclk-gpio",
			 0, &pin_i2s1mclk)) {
			if_config8 = 0;
			pr_warn("i2s1mclk-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node, "i2s1mclk-gpio",
			 1, &pin_mode_i2s1mclk)) {
			if_config8 = 0;
			pr_warn("i2s1mclk-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "i2s1ws-gpio", 0, &pin_i2s1ws)) {
			if_config9 = 0;
			pr_warn("i2s1ws-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "i2s1ws-gpio", 1, &pin_mode_i2s1ws)) {
			if_config9 = 0;
			pr_warn("i2s1ws-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "extspkamp_2-gpio", 0, &pin_extspkamp_2)) {
			if_config10 = 0;
			pr_warn("extspkamp_2-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index(node,
			 "extspkamp_2-gpio", 1, &pin_mode_extspkamp_2)) {
			if_config10 = 0;
			pr_warn("extspkamp_2-gpio get pin_mode fail!!!\n");
		}

		if (of_property_read_u32_index(node,
			 "rcvspkswitch-gpio", 0, &pin_rcvspkswitch)) {
			if_config11 = 0;
			pr_warn("rcvspkswitch-gpio get pin fail!!!\n");
		}
		if (of_property_read_u32_index
		    (node, "rcvspkswitch-gpio", 1, &pin_mode_rcvspkswitch)) {
			if_config11 = 0;
			pr_warn("rcvspkswitch-gpio get pin_mode fail!!!\n");
		}

		pr_debug("AuddrvParseGPIOaudclk=%d, miso=%d, mosi=%d\n",
		       pin_audclk, pin_audmiso, pin_audmosi);
		pr_debug("AuddrvParseGPIO vowclk=%d,extspkamp=%d\n",
			 pin_vowclk,
		       pin_extspkamp);
	} else {
		pr_err("Auddrv_OF_ParseGPIO node NULL!!!\n");
		return -1;
	}
	return 0;
}

int GetGPIO_Info(int type, int *pin, int *pinmode)
{
	switch (type) {
	case 1:		/* pin_audclk */
		if (if_config1 == 1) {
			*pin = pin_audclk | 0x80000000;
			*pinmode = pin_mode_audclk;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 2:		/* pin_audmiso */
		if (if_config2 == 1) {
			*pin = pin_audmiso | 0x80000000;
			*pinmode = pin_mode_audmiso;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 3:		/* pin_audmosi */
		if (if_config3 == 1) {
			*pin = pin_audmosi | 0x80000000;
			*pinmode = pin_mode_audmosi;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 4:		/* pin_vowclk */
		if (if_config4 == 1) {
			*pin = pin_vowclk | 0x80000000;
			*pinmode = pin_mode_vowclk;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 5:		/* pin_extspkamp */
		if (if_config5 == 1) {
			*pin = pin_extspkamp | 0x80000000;
			*pinmode = pin_mode_extspkamp;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 6:		/* pin_i2s1clk */
		if (if_config6 == 1) {
			*pin = pin_i2s1clk | 0x80000000;
			*pinmode = pin_mode_i2s1clk;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 7:		/* pin_i2s1dat */
		if (if_config7 == 1) {
			*pin = pin_i2s1dat | 0x80000000;
			*pinmode = pin_mode_i2s1dat;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 8:		/* pin_i2s1mclk */
		if (if_config8 == 1) {
			*pin = pin_i2s1mclk | 0x80000000;
			*pinmode = pin_mode_i2s1mclk;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 9:		/* pin_i2s1ws */
		if (if_config9 == 1) {
			*pin = pin_i2s1ws | 0x80000000;
			*pinmode = pin_mode_i2s1ws;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 10:		/* pin_extspkamp_2 */
		if (if_config10 == 1) {
			*pin = pin_extspkamp_2 | 0x80000000;
			*pinmode = pin_mode_extspkamp_2;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	case 11:		/* pin_rcvspkswitch */
		if (if_config11 == 1) {
			*pin = pin_rcvspkswitch | 0x80000000;
			*pinmode = pin_mode_rcvspkswitch;
		} else {
			pr_warn("GetGPIO_Info type %d fail!!!\n", type);
			*pin = -1;
			*pinmode = -1;
		}
		break;

	default:
		*pin = -1;
		*pinmode = -1;
		pr_err("Auddrv_OF_ParseGPIO invalid type=%d!!!\n", type);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(GetGPIO_Info);
#endif
#endif

static struct platform_driver mtk_afe_driver = {

	.driver = {
		   .name = MT_SOC_DL1_PCM,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = mt_soc_pcm_dl1_of_ids,
#endif
		   },
	.probe = mtk_soc_dl1_probe,
	.remove = mtk_afe_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkafe_dev;
#endif

static int __init mtk_soc_platform_init(void)
{
	int ret;

	pr_debug("%s\n", __func__);
#ifdef CONFIG_OF
	Auddrv_Reg_map_new();
	ret = Register_Aud_Irq(NULL, afe_irq_number);

#ifdef CONFIG_MTK_LEGACY
	Auddrv_OF_ParseGPIO();
#endif

#else
	soc_mtkafe_dev = platform_device_alloc(MT_SOC_DL1_PCM, -1);

	if (!soc_mtkafe_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_mtkafe_dev);
	if (ret != 0) {
		platform_device_put(soc_mtkafe_dev);
		return ret;
	}
#endif
	ret = platform_driver_register(&mtk_afe_driver);
	return ret;

}
module_init(mtk_soc_platform_init);

static void __exit mtk_soc_platform_exit(void)
{
	pr_debug("%s\n", __func__);

	platform_driver_unregister(&mtk_afe_driver);
}
module_exit(mtk_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");
