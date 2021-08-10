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
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_pcm_common.h"

/* information about */
struct AFE_MEM_CONTROL_T *VUL2_Control_context;
static struct snd_dma_buffer *Capture_dma_buf;
static struct AudioDigtalI2S *mAudioDigitalI2S;
static DEFINE_SPINLOCK(auddrv_ULInCtl_lock);

static void StartAudioCaptureHardware(struct snd_pcm_substream *substream);
static void StopAudioCaptureHardware(struct snd_pcm_substream *substream);
static int mtk_capture2_probe(struct platform_device *pdev);
static int mtk_capture2_pcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_capture2_pcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_capture2_probe(struct snd_soc_platform *platform);

static struct snd_pcm_hardware mtk_capture2_hardware = {

	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SND_SOC_STD_MT_FMTS,
	.rates = SOC_HIGH_USE_RATE,
	.rate_min = SOC_HIGH_USE_RATE_MIN,
	.rate_max = SOC_HIGH_USE_RATE_MAX,
	.channels_min = SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max = SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = UL2_MAX_BUFFER_SIZE,
	.period_bytes_max = UL2_MAX_BUFFER_SIZE,
	.periods_min = UL1_MIN_PERIOD_SIZE,
	.periods_max = UL1_MAX_PERIOD_SIZE,
	.fifo_size = 0,
};

static void StopAudioCaptureHardware(struct snd_pcm_substream *substream)
{
	pr_debug("StopAudioCaptureHardware\n");

	/* here to set interrupt */
	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, false);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC_2, false);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC_2) == false)
		Set2ndI2SAdcEnable(false);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL_DATA2, false);

	/* here to turn off digital part */
	SetConnection(Soc_Aud_InterCon_DisConnect,
		 Soc_Aud_InterConnectionInput_I17,
		      Soc_Aud_InterConnectionOutput_O21);
	SetConnection(Soc_Aud_InterCon_DisConnect,
		 Soc_Aud_InterConnectionInput_I18,
		      Soc_Aud_InterConnectionOutput_O22);

	EnableAfe(false);
}

static void ConfigAdcI2S(struct snd_pcm_substream *substream)
{
	mAudioDigitalI2S->mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
	mAudioDigitalI2S->mBuffer_Update_word = 8;
	mAudioDigitalI2S->mFpga_bit_test = 0;
	mAudioDigitalI2S->mFpga_bit = 0;
	mAudioDigitalI2S->mloopback = 0;
	mAudioDigitalI2S->mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
	mAudioDigitalI2S->mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
	mAudioDigitalI2S->mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_16BITS;
	mAudioDigitalI2S->mI2S_SAMPLERATE = (substream->runtime->rate);
}

static void StartAudioCaptureHardware(struct snd_pcm_substream *substream)
{
	pr_debug("StartAudioCaptureHardware\n");

	ConfigAdcI2S(substream);
	Set2ndI2SAdcIn(mAudioDigitalI2S);	/* To do */

	SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL_DATA2,
		 AFE_WLEN_16_BIT);
	SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
		 Soc_Aud_InterConnectionOutput_O21);
	SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
		 Soc_Aud_InterConnectionOutput_O22);

	if (GetMemoryPathEnable(
		Soc_Aud_Digital_Block_I2S_IN_ADC_2) == false) {
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC_2, true);
		Set2ndI2SAdcEnable(true);	/* To Do */
	} else
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC_2, true);

	/* here to set interrupt */
	SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE,
		 substream->runtime->period_size);
	SetIrqMcuSampleRate(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE,
		 substream->runtime->rate);
	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, true);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_VUL_DATA2,
		 substream->runtime->rate);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL_DATA2, true);

	SetConnection(Soc_Aud_InterCon_Connection,
		 Soc_Aud_InterConnectionInput_I17,
		      Soc_Aud_InterConnectionOutput_O21);
	SetConnection(Soc_Aud_InterCon_Connection,
		 Soc_Aud_InterConnectionInput_I18,
		      Soc_Aud_InterConnectionOutput_O22);
	EnableAfe(true);

}

static int mtk_capture2_pcm_prepare(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_capture2_pcm_prepare substream->rate =%d\n",
		substream->runtime->rate);
	return 0;
}

static int mtk_capture2_alsa_stop(struct snd_pcm_substream *substream)
{
	struct AFE_BLOCK_T *Vul_Block = &(VUL2_Control_context->rBlock);

	pr_debug("mtk_capture2_alsa_stop\n");
	StopAudioCaptureHardware(substream);
	Vul_Block->u4DMAReadIdx = 0;
	Vul_Block->u4WriteIdx = 0;
	Vul_Block->u4DataRemained = 0;
	RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL_DATA2, substream);
	return 0;
}

static kal_int32 Previous_Hw_cur;
static snd_pcm_uframes_t mtk_capture2_pcm_pointer(
	struct snd_pcm_substream *substream)
{
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	kal_uint32 Frameidx = 0;
	struct AFE_BLOCK_T *vul2_Block = &(VUL2_Control_context->rBlock);

	pr_debug("mtk_capture2_pcm_pointer vul2_Block->u4WriteIdx = 0x%x\n",
		vul2_Block->u4WriteIdx);
	if (VUL2_Control_context->interruptTrigger == 1) {
		/* get total bytes to copysinewavetohdmi */
		Frameidx = audio_bytes_to_frame(
			substream, vul2_Block->u4WriteIdx);
		return Frameidx;

		HW_Cur_ReadIdx = Align64ByteSize(Afe_Get_Reg(AFE_VUL_D2_CUR));
		if (HW_Cur_ReadIdx == 0) {
			pr_warn("[Auddrv] mtk_capture2_pcm_pointer HW_Cur_ReadIdx == 0\n");
			HW_Cur_ReadIdx = vul2_Block->pucPhysBufAddr;
		}
		HW_memory_index = (HW_Cur_ReadIdx - vul2_Block->pucPhysBufAddr);
		Previous_Hw_cur = HW_memory_index;
		pr_debug
			(" mtk_capture2_pcm_pointer 0x%x mem_index 0x%x\n",
			HW_Cur_ReadIdx, HW_memory_index);
		VUL2_Control_context->interruptTrigger = 0;
		return (HW_memory_index >> 2);
	}
	return (Previous_Hw_cur >> 2);

}

static void SetVULBuffer(struct snd_pcm_substream *substream,
	 struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct AFE_BLOCK_T *pblock = &VUL2_Control_context->rBlock;

	pr_debug("SetVULBuffer\n");
	pblock->pucPhysBufAddr = runtime->dma_addr;
	pblock->pucVirtBufAddr = runtime->dma_area;
	pblock->u4BufferSize = runtime->dma_bytes;
	pblock->u4SampleNumMask = 0x001f;	/* 32 byte align */
	pblock->u4WriteIdx = 0;
	pblock->u4DMAReadIdx = 0;
	pblock->u4DataRemained = 0;
	pblock->u4fsyncflag = false;
	pblock->uResetFlag = true;
	pr_debug("size = %d virAddr %p phyAddr 0x%x\n",
		pblock->u4BufferSize, pblock->pucVirtBufAddr,
		pblock->pucPhysBufAddr);

	/* set dram address top hardware */
	Afe_Set_Reg(AFE_VUL_D2_BASE, pblock->pucPhysBufAddr, 0xffffffff);
	Afe_Set_Reg(AFE_VUL_D2_END, pblock->pucPhysBufAddr +
		 (pblock->u4BufferSize - 1),
		    0xffffffff);
}

static int mtk_capture2_pcm_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
	int ret = 0;

	pr_debug("mtk_capture2_pcm_hw_params\n");

	dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buf->dev.dev = substream->pcm->card->dev;
	dma_buf->private_data = NULL;
	pr_debug("Capture2_dma_buf = %p Capture2_dma_buf->area = %p\n",
		 Capture_dma_buf,
	       Capture_dma_buf->area);

	if (Capture_dma_buf->area) {
		pr_debug("mtk_capture2_pcm_hw_params Capture_dma_buf->area\n");
		runtime->dma_bytes = Capture_dma_buf->bytes;
		runtime->dma_area = Capture_dma_buf->area;
		runtime->dma_addr = Capture_dma_buf->addr;
		runtime->buffer_size = Capture_dma_buf->bytes;
	} else {
		pr_debug("mtk_capture2_pcm_hw_params snd_pcm_lib_malloc_pages\n");
		ret = snd_pcm_lib_malloc_pages(
			substream, params_buffer_bytes(hw_params));
	}
	pr_debug("cptr2_pcm_hw_params bytes =%zu area = %p addr = 0x%lx",
	   runtime->dma_bytes, runtime->dma_area, (long)runtime->dma_addr);

	pr_debug("runtime->hw.buffer_bytes_max =%zu\n",
	 runtime->hw.buffer_bytes_max);
	SetVULBuffer(substream, hw_params);

	pr_debug("dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	       substream->runtime->dma_bytes, substream->runtime->dma_area,
	       (long)substream->runtime->dma_addr);
	return ret;
}

static int mtk_capture2_pcm_hw_free(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_capture2_pcm_hw_free\n");

	if (Capture_dma_buf->area)
		return 0;
	else
		return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {

	.count = ARRAY_SIZE(soc_normal_supported_sample_rates),
	.list = soc_normal_supported_sample_rates,
};

static int mtk_capture2_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	AudDrv_ANA_Clk_On();
	AudDrv_Clk_On();
	AudDrv_ADC2_Clk_On();

	pr_debug("%s\n", __func__);
	VUL2_Control_context = Get_Mem_ControlT(
		Soc_Aud_Digital_Block_MEM_VUL_DATA2);

	runtime->hw = mtk_capture2_hardware;
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_capture2_hardware,
	       sizeof(struct snd_pcm_hardware));
	pr_debug("runtime->hw->rates = 0x%x\n", runtime->hw.rates);

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
		 &constraints_sample_rates);
	ret = snd_pcm_hw_constraint_integer(runtime,
		 SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0)
		pr_warn("snd_pcm_hw_constraint_integer failed\n");

	pr_debug("mtk_capture2_pcm_open runtime rate = %d channels = %d\n",
		runtime->rate, runtime->channels);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		pr_debug("SNDRV_PCM_STREAM_CAPTURE mtkalsa_capture_constraints\n");

	if (ret < 0) {
		pr_warn("mtk_capture2_pcm_close\n");
		mtk_capture2_pcm_close(substream);
		return ret;
	}
	pr_debug("mtk_capture2_pcm_open return\n");
	return 0;
}

static int mtk_capture2_pcm_close(struct snd_pcm_substream *substream)
{
	AudDrv_ADC2_Clk_Off();
	AudDrv_Clk_Off();
	AudDrv_ANA_Clk_Off();
	return 0;
}

static int mtk_capture2_alsa_start(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_capture2_alsa_start\n");
	SetMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL_DATA2, substream);
	StartAudioCaptureHardware(substream);
	return 0;
}

static int mtk_capture2_pcm_trigger(struct snd_pcm_substream *substream,
	 int cmd)
{
	pr_debug("mtk_capture2_pcm_trigger cmd = %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_capture2_alsa_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_capture2_alsa_stop(substream);
	}
	return -EINVAL;
}

static bool CheckNullPointer(void *pointer)
{
	if (pointer == NULL) {
		pr_debug("CheckNullPointer pointer = NULL");
		return true;
	}
	return false;
}

static int mtk_capture2_pcm_copy(struct snd_pcm_substream *substream,
				 int channel, snd_pcm_uframes_t pos,
				 void __user *dst, snd_pcm_uframes_t count)
{

	struct AFE_MEM_CONTROL_T *pVUL_MEM_ConTrol = NULL;
	struct AFE_BLOCK_T *Vul_Block = NULL;
	char *Read_Data_Ptr = (char *)dst;
	ssize_t DMA_Read_Ptr = 0, read_size = 0, read_count = 0;
	unsigned long flags;

	pr_debug("mtk_capture2_pcm_copy pos = %lu count = %lu\n",
		 pos, count);

	/* get total bytes to copy */
	count = Align64ByteSize(audio_frame_to_bytes(substream, count));

	/* check which memif nned to be write */
	pVUL_MEM_ConTrol = VUL2_Control_context;
	Vul_Block = &(pVUL_MEM_ConTrol->rBlock);

	if (pVUL_MEM_ConTrol == NULL) {
		pr_debug("cannot find MEM control !!!!!!!\n");
		msleep(50);
		return 0;
	}

	if (Vul_Block->u4BufferSize <= 0) {
		msleep(50);
		pr_debug("Vul_Block->u4BufferSize = %d\n",
			 Vul_Block->u4BufferSize);
		return 0;
	}

	if (CheckNullPointer((void *)Vul_Block->pucVirtBufAddr)) {
		pr_debug("CheckNullPointer pucVirtBufAddr = %p\n",
			 Vul_Block->pucVirtBufAddr);
		return 0;
	}

	spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);
	if (Vul_Block->u4DataRemained > Vul_Block->u4BufferSize) {
		pr_debug("mtk_capture2_pcm_cp Rem=%x>u4BufSZ=%x\n",
			Vul_Block->u4DataRemained, Vul_Block->u4BufferSize);
		Vul_Block->u4DataRemained = 0;
		Vul_Block->u4DMAReadIdx = Vul_Block->u4WriteIdx;
	}

	if (count > Vul_Block->u4DataRemained)
		read_size = Vul_Block->u4DataRemained;
	else
		read_size = count;

	DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
	spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);

	if (DMA_Read_Ptr + read_size < Vul_Block->u4BufferSize) {
		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {
			pr_debug
			("1,rdsz:%zu,Rem:%x,DMARd:%zu,DMARdId:%x\n",
				read_size, Vul_Block->u4DataRemained,
				DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
		}

		if (copy_to_user((void __user *)Read_Data_Ptr,
			(Vul_Block->pucVirtBufAddr + DMA_Read_Ptr),
			read_size)) {
			pr_err("%s Fail 1 copy to user rp:%p, virAddr:%p,",
				__func__, Read_Data_Ptr,
				Vul_Block->pucVirtBufAddr);
			pr_err("rp:0x%x, DMA_Rd_Ptr:%zu, size:%zu",
				Vul_Block->u4DMAReadIdx,
				DMA_Read_Ptr, read_size);
			return 0;
		}

		read_count += read_size;
		spin_lock(&auddrv_ULInCtl_lock);
		Vul_Block->u4DataRemained -= read_size;
		Vul_Block->u4DMAReadIdx += read_size;
		Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
		DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
		spin_unlock(&auddrv_ULInCtl_lock);

		Read_Data_Ptr += read_size;
		count -= read_size;

		pr_debug
			("%s finish1, size:%zu, rp:%x, wp:%x,remained:%x\n",
			__func__, read_size, Vul_Block->u4DMAReadIdx,
			Vul_Block->u4WriteIdx,
			Vul_Block->u4DataRemained);
	} else {
		uint32 size_1 = Vul_Block->u4BufferSize - DMA_Read_Ptr;
		uint32 size_2 = read_size - size_1;

		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {
		pr_debug
			("%s 2, size_1:%x, remained:%x,DMA_Ptr:%zu,rp:%x\n",
			__func__, size_1, Vul_Block->u4DataRemained,
			 DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
		}
		if (copy_to_user((void __user *)Read_Data_Ptr,
			(Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), size_1)) {
		pr_err("%s Fail 2 cp to user Rd_Ptr:%p, pucAddr:%p,",
			__func__, Read_Data_Ptr, Vul_Block->pucVirtBufAddr);
		pr_err(" rp:0x%x, DMA_Read_Ptr:%zu,size:%zu\n",
			Vul_Block->u4DMAReadIdx,
			DMA_Read_Ptr, read_size);
		return 0;
		}

		read_count += size_1;
		spin_lock(&auddrv_ULInCtl_lock);
		Vul_Block->u4DataRemained -= size_1;
		Vul_Block->u4DMAReadIdx += size_1;
		Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
		DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
		spin_unlock(&auddrv_ULInCtl_lock);


		pr_debug
		("%s end2, cp_sz_1:%x, DMARdId:%x, WtId:%x, Rem:%x\n",
		__func__, size_1, Vul_Block->u4DMAReadIdx,
		Vul_Block->u4WriteIdx,
		Vul_Block->u4DataRemained);

		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {
			pr_debug
			("%s 3, rdSz:%x Rem:%x,DMA_Ptr:%zu, DMARdId:%x\n",
			__func__, size_2, Vul_Block->u4DataRemained
			, DMA_Read_Ptr,
			Vul_Block->u4DMAReadIdx);
		}
		if (copy_to_user((void __user *)(Read_Data_Ptr + size_1),
		     (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), size_2)) {
		pr_err("%s Fail 3 cp user RdPtr:%p, pucAddr:%p,",
			 __func__, Read_Data_Ptr,
			 Vul_Block->pucVirtBufAddr);
		pr_err("DMARdId:0x%x,DMARdPtr:%zu,read_size:%zu\n",
			Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
			return read_count << 2;
		}

		read_count += size_2;
		spin_lock(&auddrv_ULInCtl_lock);
		Vul_Block->u4DataRemained -= size_2;
		Vul_Block->u4DMAReadIdx += size_2;
		DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
		spin_unlock(&auddrv_ULInCtl_lock);

		count -= read_size;
		Read_Data_Ptr += read_size;

		pr_debug
		("%s end3, cp size_2:%x, DMAId%x,WrtId%x, Remain:%x\n",
		__func__, size_2, Vul_Block->u4DMAReadIdx,
		 Vul_Block->u4WriteIdx,
		Vul_Block->u4DataRemained);
	}

	return read_count >> 2;
}

static int mtk_capture2_pcm_silence(struct snd_pcm_substream *substream,
	int channel, snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	pr_debug("dummy_pcm_silence\n");
	/* do nothing */
	return 0;
}


static void *dummy_page[2];

static struct page *mtk_capture2_pcm_page(
	struct snd_pcm_substream *substream, unsigned long offset)
{
	pr_debug("%s\n", __func__);
	return virt_to_page(dummy_page[substream->stream]);
}


static struct snd_pcm_ops mtk_afe_capture2_ops = {

	.open = mtk_capture2_pcm_open,
	.close = mtk_capture2_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mtk_capture2_pcm_hw_params,
	.hw_free = mtk_capture2_pcm_hw_free,
	.prepare = mtk_capture2_pcm_prepare,
	.trigger = mtk_capture2_pcm_trigger,
	.pointer = mtk_capture2_pcm_pointer,
	.copy = mtk_capture2_pcm_copy,
	.silence = mtk_capture2_pcm_silence,
	.page = mtk_capture2_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform = {

	.ops = &mtk_afe_capture2_ops,
	.pcm_new = mtk_asoc_capture2_pcm_new,
	.probe = mtk_afe_capture2_probe,
};

static int mtk_capture2_probe(struct platform_device *pdev)
{
	pr_debug("mtk_capture2_probe\n");

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);

	if (pdev->dev.dma_mask == NULL)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_UL2_PCM);

	pr_debug("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev, &mtk_soc_platform);
}

static int mtk_asoc_capture2_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	pr_debug("mtk_asoc_capture2_pcm_new\n");
	return 0;
}


static int mtk_afe_capture2_probe(struct snd_soc_platform *platform)
{
	pr_debug("mtk_afe_capture2_probe\n");
	AudDrv_Allocate_mem_Buffer(platform->dev,
		 Soc_Aud_Digital_Block_MEM_VUL_DATA2,
		   UL2_MAX_BUFFER_SIZE);
	Capture_dma_buf = Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_VUL_DATA2);
	mAudioDigitalI2S = kzalloc(sizeof(struct AudioDigtalI2S), GFP_KERNEL);
	return 0;
}


static int mtk_capture2_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_capture2_of_ids[] = {

	{.compatible = "mediatek,mt8163-soc-pcm-capture2",},
	{}
};
#endif

static struct platform_driver mtk_afe_capture2_driver = {

	.driver = {
		   .name = MT_SOC_UL2_PCM,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = mt_soc_pcm_capture2_of_ids,
#endif
		   },
	.probe = mtk_capture2_probe,
	.remove = mtk_capture2_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkafe_capture2_dev;
#endif

static int __init mtk_soc_capture2_platform_init(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);
#ifndef CONFIG_OF
	soc_mtkafe_capture2_dev = platform_device_alloc(MT_SOC_UL2_PCM, -1);

	if (!soc_mtkafe_capture2_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_mtkafe_capture2_dev);
	if (ret != 0) {
		platform_device_put(soc_mtkafe_capture2_dev);
		return ret;
	}
#endif

	ret = platform_driver_register(&mtk_afe_capture2_driver);
	return ret;
}
module_init(mtk_soc_capture2_platform_init);

static void __exit mtk_soc_capture2_platform_exit(void)
{

	pr_debug("%s\n", __func__);
	platform_driver_unregister(&mtk_afe_capture2_driver);
}
module_exit(mtk_soc_capture2_platform_exit);

MODULE_DESCRIPTION("AFE Capture2 module platform driver");
MODULE_LICENSE("GPL");
