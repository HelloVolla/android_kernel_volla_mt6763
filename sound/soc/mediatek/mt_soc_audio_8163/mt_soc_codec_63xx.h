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



#ifndef _AUDIO_CODEC_63xx_H
#define _AUDIO_CODEC_63xx_H

void audckbufEnable(bool enable);
void ClsqEnable(bool enable);
void Topck_Enable(bool enable);
void NvregEnable(bool enable);
void OpenClassAB(void);
void OpenAnalogHeadphone(bool bEnable);
void OpenAnalogTrimHardware(bool bEnable);
void SetSdmLevel(unsigned int level);
void setOffsetTrimMux(unsigned int Mux);
void setOffsetTrimBufferGain(unsigned int gain);
void EnableTrimbuffer(bool benable);
void SetHplTrimOffset(int Offset);
void SetHprTrimOffset(int Offset);
void setHpGainZero(void);
bool OpenHeadPhoneImpedanceSetting(bool bEnable);
void SetAnalogSuspend(bool bEnable);
void OpenTrimBufferHardware(bool bEnable);
void setHpDcCalibration(unsigned int type, int dc_cali_value);
void setHpDcCalibrationGain(unsigned int type, int gain_value);


#endif

