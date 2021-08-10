/*************************************************************************/ /*!
@File
@Title          BVNC handling specific routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Functions used for BNVC related work
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include "img_defs.h"
#include "rgxbvnc.h"
#define _RGXBVNC_C_
#include "rgx_bvnc_table_km.h"
#undef _RGXBVNC_C_
#include "oskm_apphint.h"
#include "pvrsrv.h"

#define RGXBVNC_BUFFER_SIZE (((PVRSRV_MAX_DEVICES)*(RGX_BVNC_STR_SIZE_MAX))+1)

/* This function searches the given array for a given search value */
static IMG_UINT64* _RGXSearchBVNCTable( IMG_UINT64 *pui64Array,
								IMG_UINT uiEnd,
								IMG_UINT64 ui64SearchValue,
								IMG_UINT uiRowCount)
{
	IMG_UINT uiStart = 0, index;
	IMG_UINT64 value, *pui64Ptr = NULL;

	while (uiStart < uiEnd)
	{
		index = (uiStart + uiEnd)/2;
		pui64Ptr = pui64Array + (index * uiRowCount);
		value = *(pui64Ptr);

		if (value == ui64SearchValue)
		{
			return pui64Ptr;
		}

		if (value > ui64SearchValue)
		{
			uiEnd = index;
		}else
		{
			uiStart = index + 1;
		}
	}
	return NULL;
}
#define RGX_SEARCH_BVNC_TABLE(t, b) (_RGXSearchBVNCTable((IMG_UINT64*)(t), \
                                ARRAY_SIZE(t), (b), \
                                sizeof((t)[0])/sizeof(IMG_UINT64)) )


#if defined(DEBUG)

#define PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, szShortName, Feature)															\
	if ( psDevInfo->sDevFeatureCfg.ui32FeaturesValues[RGX_FEATURE_##Feature##_IDX] != RGX_FEATURE_VALUE_DISABLED )			\
		{ PVR_LOG(("%s %d", szShortName, psDevInfo->sDevFeatureCfg.ui32FeaturesValues[RGX_FEATURE_##Feature##_IDX])); }		\
	else																\
		{ PVR_LOG(("%s N/A", szShortName)); }

static void _RGXBvncDumpParsedConfig(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = (PVRSRV_RGXDEV_INFO *)psDeviceNode->pvDevice;
	IMG_UINT64 ui64Mask = 0, ui32IdOrNameIdx = 1;

	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "NC:       ", NUM_CLUSTERS);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "CSF:      ", CDM_CONTROL_STREAM_FORMAT);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "FBCDCA:   ", FBCDC_ARCHITECTURE);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "MCMB:     ", META_COREMEM_BANKS);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "MCMS:     ", META_COREMEM_SIZE);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "MDMACnt:  ", META_DMA_CHANNEL_COUNT);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "NIIP:     ", NUM_ISP_IPP_PIPES);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "PBW:      ", PHYS_BUS_WIDTH);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "STEArch:  ", SCALABLE_TE_ARCH);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "SVCEA:    ", SCALABLE_VCE);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "SLCBanks: ", SLC_BANKS);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "SLCCLS:   ", SLC_CACHE_LINE_SIZE_BITS);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "SLCSize:  ", SLC_SIZE_IN_BYTES);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "VASB:     ", VIRTUAL_ADDRESS_SPACE_BITS);
	PVR_LOG_DUMP_FEATURE_VALUE(psDevInfo, "META:     ", META);

#if defined(FEATURE_NO_VALUES_NAMES_MAX_IDX)
	/* Dump the features with no values */
	ui64Mask = psDevInfo->sDevFeatureCfg.ui64Features;
	while (ui64Mask)
	{
		if (ui64Mask & 0x01)
		{
			if (ui32IdOrNameIdx <= FEATURE_NO_VALUES_NAMES_MAX_IDX)
			{
				PVR_LOG(("%s", gaszFeaturesNoValuesNames[ui32IdOrNameIdx - 1]));
			}
			else
			{
				PVR_DPF((PVR_DBG_WARNING,"Feature with Mask doesn't exist: 0x%016" IMG_UINT64_FMTSPECx, ((IMG_UINT64)1 << (ui32IdOrNameIdx - 1))));
			}
		}
		ui64Mask >>= 1;
		ui32IdOrNameIdx++;
	}
#endif

#if defined(ERNSBRNS_IDS_MAX_IDX)
	/* Dump the ERN and BRN flags for this core */
	ui64Mask = psDevInfo->sDevFeatureCfg.ui64ErnsBrns;
	ui32IdOrNameIdx = 1;

	while (ui64Mask)
	{
		if (ui64Mask & 0x1)
		{
			if (ui32IdOrNameIdx <= ERNSBRNS_IDS_MAX_IDX)
			{
				PVR_LOG(("ERN/BRN : %d", gaui64ErnsBrnsIDs[ui32IdOrNameIdx - 1]));
			}
			else
			{
				PVR_LOG(("Unknown ErnBrn bit: 0x%0" IMG_UINT64_FMTSPECx, ((IMG_UINT64)1 << (ui32IdOrNameIdx - 1))));
			}
		}
		ui64Mask >>= 1;
		ui32IdOrNameIdx++;
	}
#endif

}
#endif

static void _RGXBvncParseFeatureValues(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_UINT64 ui64PackedValues)
{
	IMG_UINT32 ui32Index;

	/* Read the feature values for the runtime BVNC */
	for (ui32Index = 0; ui32Index < RGX_FEATURE_WITH_VALUES_MAX_IDX; ui32Index++)
	{
		IMG_UINT16	ui16ValueIndex = (ui64PackedValues & aui64FeaturesWithValuesBitMasks[ui32Index]) >> aui16FeaturesWithValuesBitPositions[ui32Index];

		if (ui16ValueIndex < gaFeaturesValuesMaxIndexes[ui32Index])
		{
			if (gaFeaturesValues[ui32Index][ui16ValueIndex] == (IMG_UINT16)RGX_FEATURE_VALUE_DISABLED)
			{
				psDevInfo->sDevFeatureCfg.ui32FeaturesValues[ui32Index] = RGX_FEATURE_VALUE_DISABLED;
			}
			else
			{
				psDevInfo->sDevFeatureCfg.ui32FeaturesValues[ui32Index] = gaFeaturesValues[ui32Index][ui16ValueIndex];
			}
		}
		else
		{
			/* This case should never be reached */
			psDevInfo->sDevFeatureCfg.ui32FeaturesValues[ui32Index] = RGX_FEATURE_VALUE_INVALID;
			PVR_DPF((PVR_DBG_ERROR, "%s: Feature with index (%d) decoded wrong value index (%d)", __func__, ui32Index, ui16ValueIndex));
			PVR_ASSERT(ui16ValueIndex < gaFeaturesValuesMaxIndexes[ui32Index]);
		}
	}

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, MIPS))
	{
		psDevInfo->sDevFeatureCfg.ui32FeaturesValues[RGX_FEATURE_META_IDX] = RGX_FEATURE_VALUE_DISABLED;
	}

	/* Get the max number of dusts in the core */
	if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, NUM_CLUSTERS))
	{
		psDevInfo->sDevFeatureCfg.ui32MAXDustCount = MAX(1, (RGX_GET_FEATURE_VALUE(psDevInfo, NUM_CLUSTERS) / 2));
	}
	else
	{
		/* This case should never be reached as all cores have clusters */
		psDevInfo->sDevFeatureCfg.ui32MAXDustCount = RGX_FEATURE_VALUE_INVALID;
		PVR_DPF((PVR_DBG_ERROR, "%s: Number of clusters feature value missing!", __func__));
		PVR_ASSERT(0);
	}


	/* Transform the SLC cacheline size info in bytes */
	if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, SLC_SIZE_IN_BYTES))
	{
		psDevInfo->sDevFeatureCfg.ui32FeaturesValues[RGX_FEATURE_SLC_SIZE_IN_BYTES_IDX] *= 1024;
	}

	/* Transform the META coremem size info in bytes */
	if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, META_COREMEM_SIZE))
	{
		psDevInfo->sDevFeatureCfg.ui32FeaturesValues[RGX_FEATURE_META_COREMEM_SIZE_IDX] *= 1024;
	}
}

static void _RGXBvncAcquireAppHint(IMG_CHAR *pszBVNC, const IMG_UINT32 ui32RGXDevCount)
{
	IMG_CHAR *pszAppHintDefault = PVRSRV_APPHINT_RGXBVNC;
	void *pvAppHintState = NULL;
	IMG_UINT32 ui32BVNCCount = 0;
	IMG_BOOL bRet;
	IMG_CHAR szBVNCAppHint[RGXBVNC_BUFFER_SIZE];
	IMG_CHAR *pszCurrentBVNC = szBVNCAppHint;
	szBVNCAppHint[0] = '\0';

	OSCreateKMAppHintState(&pvAppHintState);

	bRet = (IMG_BOOL)OSGetKMAppHintSTRING(pvAppHintState,
						RGXBVNC,
						&pszAppHintDefault,
						szBVNCAppHint,
						sizeof(szBVNCAppHint));

	OSFreeKMAppHintState(pvAppHintState);

	if (!bRet || (szBVNCAppHint[0] == '\0'))
	{
		return;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "%s: BVNC module param list: %s",__func__, szBVNCAppHint));

	while (*pszCurrentBVNC != '\0')
	{
		IMG_CHAR *pszNext = pszCurrentBVNC;

		if (ui32BVNCCount >= PVRSRV_MAX_DEVICES)
		{
			break;
		}

		while (1)
		{
			if (*pszNext == ',')
			{
				pszNext[0] = '\0';
				pszNext++;
				break;
			} else if (*pszNext == '\0')
			{
				break;
			}
			pszNext++;
		}

		if (ui32BVNCCount == ui32RGXDevCount)
		{
			strcpy(pszBVNC, pszCurrentBVNC);
			return;
		}

		ui32BVNCCount++;
		pszCurrentBVNC = pszNext;
	}

	PVR_DPF((PVR_DBG_ERROR, "%s: Given module parameters list is shorter than "
	"number of actual devices", __func__));

	/* If only one BVNC parameter is specified, the same is applied for all RGX
	 * devices detected */
	if (1 == ui32BVNCCount)
	{
		strcpy(pszBVNC, szBVNCAppHint);
	}
}

/* Function that parses the BVNC List passed as module parameter */
static PVRSRV_ERROR _RGXBvncParseList(IMG_UINT32 *pB,
									  IMG_UINT32 *pV,
									  IMG_UINT32 *pN,
									  IMG_UINT32 *pC,
									  const IMG_UINT32 ui32RGXDevCount)
{
	unsigned int ui32ScanCount = 0;
	IMG_CHAR aszBVNCString[RGX_BVNC_STR_SIZE_MAX];

	aszBVNCString[0] = '\0';

	/* 4 components of a BVNC string is B, V, N & C */
#define RGX_BVNC_INFO_PARAMS (4)

	_RGXBvncAcquireAppHint(aszBVNCString, ui32RGXDevCount);

	if ('\0' == aszBVNCString[0])
	{
		return PVRSRV_ERROR_INVALID_BVNC_PARAMS;
	}

	/* Parse the given RGX_BVNC string */
	ui32ScanCount = OSVSScanf(aszBVNCString, RGX_BVNC_STR_FMTSPEC, pB, pV, pN, pC);
	if (RGX_BVNC_INFO_PARAMS != ui32ScanCount)
	{
		ui32ScanCount = OSVSScanf(aszBVNCString,RGX_BVNC_STRP_FMTSPEC, pB, pV, pN, pC);
	}
	if (RGX_BVNC_INFO_PARAMS != ui32ScanCount)
	{
		return PVRSRV_ERROR_INVALID_BVNC_PARAMS;
	}
	PVR_LOG(("BVNC module parameter honoured: %s", aszBVNCString));

	return PVRSRV_OK;
}

/* This function detects the Rogue variant and configures the
 * essential config info associated with such a device.
 * The config info includes features, errata, etc */
PVRSRV_ERROR RGXBvncInitialiseConfiguration(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	static IMG_UINT32 ui32RGXDevCnt = 0;
	PVRSRV_ERROR eError;
	PVRSRV_RGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
	IMG_UINT64 ui64BVNC=0;
	IMG_UINT32 B=0, V=0, N=0, C=0;
	IMG_UINT64 *pui64Cfg = NULL;

	/* Check for load time RGX BVNC parameter */
	eError = _RGXBvncParseList(&B,&V,&N,&C, ui32RGXDevCnt);
	if (PVRSRV_OK == eError)
	{
		PVR_LOG(("Read BVNC " RGX_BVNC_STR_FMTSPEC \
				" from driver load parameter", B, V, N, C));

		/* Extract the BVNC config from the Features table */
		ui64BVNC = BVNC_PACK(B,0,N,C);
		pui64Cfg = RGX_SEARCH_BVNC_TABLE(gaFeatures, ui64BVNC);
		PVR_LOG_IF_FALSE((pui64Cfg != NULL), "Driver parameter BVNC configuration not found!");
	}

	{
		void *pvAppHintState = NULL;
		IMG_BOOL bAppHintDefault = PVRSRV_APPHINT_IGNOREHWREPORTEDBVNC;

		OSCreateKMAppHintState(&pvAppHintState);
		OSGetKMAppHintBOOL(pvAppHintState,
							IgnoreHWReportedBVNC,
							&bAppHintDefault,
							&psDevInfo->bIgnoreHWReportedBVNC);
		OSFreeKMAppHintState(pvAppHintState);
	}

#if !defined(NO_HARDWARE) && defined(SUPPORT_MULTIBVNC_RUNTIME_BVNC_ACQUISITION)

	/* Try to detect the RGX BVNC from the HW device */
	if ((NULL == pui64Cfg) && !PVRSRV_VZ_MODE_IS(DRIVER_MODE_GUEST) && !psDevInfo->bIgnoreHWReportedBVNC)
	{
		IMG_UINT64 ui32ID;
		IMG_HANDLE hSysData;

		hSysData = psDeviceNode->psDevConfig->hSysData;

		/* Power-up the device as required to read the registers */
		if (psDeviceNode->psDevConfig->pfnPrePowerState)
		{
			eError = psDeviceNode->psDevConfig->pfnPrePowerState(hSysData, PVRSRV_DEV_POWER_STATE_ON,
					PVRSRV_DEV_POWER_STATE_OFF, IMG_FALSE);
			PVR_LOGR_IF_ERROR(eError, "pfnPrePowerState ON");
		}

		if (psDeviceNode->psDevConfig->pfnPostPowerState)
		{
			eError = psDeviceNode->psDevConfig->pfnPostPowerState(hSysData, PVRSRV_DEV_POWER_STATE_ON,
					PVRSRV_DEV_POWER_STATE_OFF, IMG_FALSE);
			PVR_LOGR_IF_ERROR(eError, "pfnPostPowerState ON");
		}

		/* Read the BVNC, in to new way first, if B not set, use old scheme */
		ui32ID = OSReadHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_CORE_ID__PBVNC);

		if (GET_B(ui32ID))
		{
			B = (ui32ID & ~RGX_CR_CORE_ID__PBVNC__BRANCH_ID_CLRMSK) >>
													RGX_CR_CORE_ID__PBVNC__BRANCH_ID_SHIFT;
			V = (ui32ID & ~RGX_CR_CORE_ID__PBVNC__VERSION_ID_CLRMSK) >>
													RGX_CR_CORE_ID__PBVNC__VERSION_ID_SHIFT;
			N = (ui32ID & ~RGX_CR_CORE_ID__PBVNC__NUMBER_OF_SCALABLE_UNITS_CLRMSK) >>
													RGX_CR_CORE_ID__PBVNC__NUMBER_OF_SCALABLE_UNITS_SHIFT;
			C = (ui32ID & ~RGX_CR_CORE_ID__PBVNC__CONFIG_ID_CLRMSK) >>
													RGX_CR_CORE_ID__PBVNC__CONFIG_ID_SHIFT;

		}
		else
		{
			IMG_UINT64 ui32CoreID, ui32CoreRev;
			ui32CoreRev = OSReadHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_CORE_REVISION);
			ui32CoreID = OSReadHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_CORE_ID);
			B = (ui32CoreRev & ~RGX_CR_CORE_REVISION_MAJOR_CLRMSK) >>
													RGX_CR_CORE_REVISION_MAJOR_SHIFT;
			V = (ui32CoreRev & ~RGX_CR_CORE_REVISION_MINOR_CLRMSK) >>
													RGX_CR_CORE_REVISION_MINOR_SHIFT;
			N = (ui32CoreID & ~RGX_CR_CORE_ID_CONFIG_N_CLRMSK) >>
													RGX_CR_CORE_ID_CONFIG_N_SHIFT;
			C = (ui32CoreID & ~RGX_CR_CORE_ID_CONFIG_C_CLRMSK) >>
													RGX_CR_CORE_ID_CONFIG_C_SHIFT;
		}
		PVR_LOG(("Read BVNC " RGX_BVNC_STR_FMTSPEC \
				" from HW device registers",	B, V, N, C));

		/* Power-down the device */
		if (psDeviceNode->psDevConfig->pfnPrePowerState)
		{
			eError = psDeviceNode->psDevConfig->pfnPrePowerState(hSysData, PVRSRV_DEV_POWER_STATE_OFF,
					PVRSRV_DEV_POWER_STATE_ON, IMG_FALSE);
			PVR_LOGR_IF_ERROR(eError, "pfnPrePowerState OFF");
		}

		if (psDeviceNode->psDevConfig->pfnPostPowerState)
		{
			eError = psDeviceNode->psDevConfig->pfnPostPowerState(hSysData, PVRSRV_DEV_POWER_STATE_OFF,
					PVRSRV_DEV_POWER_STATE_ON, IMG_FALSE);
			PVR_LOGR_IF_ERROR(eError, "pfnPostPowerState OFF");
		}

		/* Extract the BVNC config from the Features table */
		ui64BVNC = BVNC_PACK(B,0,N,C);
		pui64Cfg = RGX_SEARCH_BVNC_TABLE(gaFeatures, ui64BVNC);
		PVR_LOG_IF_FALSE((pui64Cfg != NULL), "HW device BVNC configuration not found!");
	}
#endif

	/* We reach here if the HW is not present, or we are running in a guest OS,
	 * or HW is unstable during register read giving invalid values, or
	 * runtime detection has been disabled - fall back to compile time BVNC */
	if (NULL == pui64Cfg)
	{
		B = RGX_BVNC_KM_B;
		N = RGX_BVNC_KM_N;
		C = RGX_BVNC_KM_C;
		{
			IMG_UINT32	ui32ScanCount = 0;
			ui32ScanCount = OSVSScanf(RGX_BVNC_KM_V_ST, "%u", &V);
			if (1 != ui32ScanCount)
			{
				ui32ScanCount = OSVSScanf(RGX_BVNC_KM_V_ST, "%up", &V);
				if (1 != ui32ScanCount)
				{
					V = 0;
				}
			}
		}
		PVR_LOG(("Reverting to compile time BVNC %s", RGX_BVNC_KM));

		/* Extract the BVNC config from the Features table */
		ui64BVNC = BVNC_PACK(B,0,N,C);
		pui64Cfg = RGX_SEARCH_BVNC_TABLE(gaFeatures, ui64BVNC);
		PVR_LOG_IF_FALSE((pui64Cfg != NULL), "Compile time BVNC configuration not found!");
	}

	/* Have we failed to identify the BVNC to use? */
	if (NULL == pui64Cfg)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: BVNC Detection and feature lookup failed. "
		    "Unsupported BVNC: 0x%016" IMG_UINT64_FMTSPECx, __func__, ui64BVNC));
		return PVRSRV_ERROR_BVNC_UNSUPPORTED;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "%s: BVNC Feature found config: 0x%016"
	    IMG_UINT64_FMTSPECx " 0x%016" IMG_UINT64_FMTSPECx " 0x%016"
	    IMG_UINT64_FMTSPECx "\n",__func__, pui64Cfg[0], pui64Cfg[1],
	    pui64Cfg[2]));

	/* Parsing feature config depends on available features on the core
	 * hence this parsing should always follow the above feature assignment */
	psDevInfo->sDevFeatureCfg.ui64Features = pui64Cfg[1];
	_RGXBvncParseFeatureValues(psDevInfo, pui64Cfg[2]);

	/* Add 'V' to the packed BVNC value to get the BVNC ERN and BRN config. */
	ui64BVNC = BVNC_PACK(B,V,N,C);
	pui64Cfg = RGX_SEARCH_BVNC_TABLE(gaErnsBrns, ui64BVNC);
	if (NULL == pui64Cfg)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: BVNC ERN/BRN lookup failed. "
		    "Unsupported BVNC: 0x%016" IMG_UINT64_FMTSPECx,	__func__, ui64BVNC));
		psDevInfo->sDevFeatureCfg.ui64ErnsBrns = 0;
		return PVRSRV_ERROR_BVNC_UNSUPPORTED;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "%s: BVNC ERN/BRN Cfg: 0x%016" IMG_UINT64_FMTSPECx
	    " 0x%016" IMG_UINT64_FMTSPECx, __func__, *pui64Cfg, pui64Cfg[1]));
	psDevInfo->sDevFeatureCfg.ui64ErnsBrns = pui64Cfg[1];

	psDevInfo->sDevFeatureCfg.ui32B = B;
	psDevInfo->sDevFeatureCfg.ui32V = V;
	psDevInfo->sDevFeatureCfg.ui32N = N;
	psDevInfo->sDevFeatureCfg.ui32C = C;

	/* Message to confirm configuration look up was a success */
	PVR_LOG(("RGX Device registered with BVNC " RGX_BVNC_STR_FMTSPEC, \
			B, V, N, C));

	ui32RGXDevCnt++;

#if defined(DEBUG)
	_RGXBvncDumpParsedConfig(psDeviceNode);
#endif
	return PVRSRV_OK;
}

/*
 * This function checks if a particular feature is available on the given rgx device */
IMG_BOOL RGXBvncCheckFeatureSupported(PVRSRV_DEVICE_NODE *psDeviceNode, IMG_UINT64 ui64FeatureMask)
{
	PVRSRV_RGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
	/* FIXME: need to implement a bounds check for passed feature mask */
	if (psDevInfo->sDevFeatureCfg.ui64Features & ui64FeatureMask)
	{
		return IMG_TRUE;
	}
	return IMG_FALSE;
}

/*
 * This function returns the value of a feature on the given rgx device */
IMG_INT32 RGXBvncGetSupportedFeatureValue(PVRSRV_DEVICE_NODE *psDeviceNode, RGX_FEATURE_WITH_VALUE_INDEX eFeatureIndex)
{
	PVRSRV_RGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
	/*FIXME: need to implement a bounds check for passed feature mask */

	if (eFeatureIndex >= RGX_FEATURE_WITH_VALUES_MAX_IDX)
	{
		return -1;
	}

	if (psDevInfo->sDevFeatureCfg.ui32FeaturesValues[eFeatureIndex] == RGX_FEATURE_VALUE_DISABLED)
	{
		return -1;
	}

	return psDevInfo->sDevFeatureCfg.ui32FeaturesValues[eFeatureIndex];
}
