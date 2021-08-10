/*************************************************************************/ /*!
@File
@Title          Services initialisation routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Device specific functions
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
#include "srvinit.h"
#include "pvr_debug.h"
#include "osfunc.h"
#include "km_apphint_defs.h"
#include "htbuffer_types.h"
#include "htbuffer_init.h"

#include "devicemem.h"
#include "devicemem_pdump.h"

#include "rgx_fwif_km.h"
#include "pdump_km.h"

#include "rgxinit.h"

#include "rgx_compat_bvnc.h"

#include "osfunc.h"

#include "rgxdefs_km.h"

#if defined(SUPPORT_GPUVIRT_VALIDATION)
#include "virt_validation_defs.h"
#endif

#include "rgx_fwif_hwperf.h"
#include "rgx_hwperf_table.h"

#include "rgxfwload.h"
#include "rgxlayer_impl.h"
#include "rgxfwimageutils.h"
#include "rgxfwutils.h"

#include "rgx_hwperf.h"
#include "rgx_bvnc_defs_km.h"

#include "rgxdevice.h"
#include "img_opts.h"

#include "pvrsrv.h"

#if defined(SUPPORT_TRUSTED_DEVICE)
#include "rgxdevice.h"
#include "pvrsrv_device.h"
#endif

#define DRIVER_MODE_HOST               0          /* AppHint value for host driver mode */

#define	HW_PERF_FILTER_DEFAULT         0x00000000 /* Default to no HWPerf */
#define HW_PERF_FILTER_DEFAULT_ALL_ON  0xFFFFFFFF /* All events */

#if defined(SUPPORT_VALIDATION)
#include "pvrsrv_apphint.h"
#endif

#include "os_srvinit_param.h"
#if !defined(LINUX)
/*!
*******************************************************************************
 * AppHint mnemonic data type helper tables
******************************************************************************/
/* apphint map of name vs. enable flag */
static SRV_INIT_PARAM_UINT32_LOOKUP htb_loggroup_tbl[] = {
#define X(a, b) { #b, HTB_LOG_GROUP_FLAG(a) },
	HTB_LOG_SFGROUPLIST
#undef X
};
/* apphint map of arg vs. OpMode */
static SRV_INIT_PARAM_UINT32_LOOKUP htb_opmode_tbl[] = {
	{ "droplatest", HTB_OPMODE_DROPLATEST},
	{ "dropoldest", HTB_OPMODE_DROPOLDEST},
	/* HTB should never be started in HTB_OPMODE_BLOCK
	 * as this can lead to deadlocks
	 */
};

static SRV_INIT_PARAM_UINT32_LOOKUP fwt_logtype_tbl[] = {
	{ "trace", 2},
	{ "tbi", 1},
	{ "none", 0}
};

static SRV_INIT_PARAM_UINT32_LOOKUP timecorr_clk_tbl[] = {
	{ "mono", 0 },
	{ "mono_raw", 1 },
	{ "sched", 2 }
};

static SRV_INIT_PARAM_UINT32_LOOKUP fwt_loggroup_tbl[] = { RGXFWIF_LOG_GROUP_NAME_VALUE_MAP };

/*
 * Services AppHints initialisation
 */
#define X(a, b, c, d, e) SrvInitParamInit ## b(a, d, e)
APPHINT_LIST_ALL
#undef X
#endif /* !defined(LINUX) */

/*
 * Container for all the apphints used by this module
 */
typedef struct _RGX_SRVINIT_APPHINTS_
{
	IMG_UINT32 ui32DriverMode;
	IMG_BOOL   bDustRequestInject;
	IMG_BOOL   bEnableSignatureChecks;
	IMG_UINT32 ui32SignatureChecksBufSize;

#if defined(DEBUG)
	IMG_BOOL   bAssertOnOutOfMem;
#endif
#if defined(SUPPORT_VALIDATION)
	IMG_BOOL   bValidateIrq;
#endif
	IMG_BOOL   bAssertOnHWRTrigger;
#if defined(SUPPORT_VALIDATION)
	IMG_UINT32 aui32TPUTrilinearFracMask[RGXFWIF_TPU_DM_LAST];
	IMG_UINT32 ui32FBCDCVersionOverride;
#endif
	IMG_BOOL   bCheckMlist;
	IMG_BOOL   bDisableClockGating;
	IMG_BOOL   bDisableDMOverlap;
	IMG_BOOL   bDisableFEDLogging;
	IMG_BOOL   bDisablePDP;
	IMG_BOOL   bEnableCDMKillRand;
	IMG_BOOL   bEnableHWR;
	IMG_BOOL   bFilteringMode;
	IMG_BOOL   bHWPerfDisableCustomCounterFilter;
	IMG_BOOL   bZeroFreelist;
	IMG_UINT32 ui32EnableFWContextSwitch;
	IMG_UINT32 ui32FWContextSwitchProfile;
	IMG_UINT32 ui32VDMContextSwitchMode;
	IMG_UINT32 ui32HWPerfFWBufSize;
	IMG_UINT32 ui32HWPerfHostBufSize;
	IMG_UINT32 ui32HWPerfFilter0;
	IMG_UINT32 ui32HWPerfFilter1;
	IMG_UINT32 ui32HWPerfHostFilter;
	IMG_UINT32 ui32TimeCorrClock;
	IMG_UINT32 ui32HWRDebugDumpLimit;
	IMG_UINT32 ui32JonesDisableMask;
	IMG_UINT32 ui32LogType;
	IMG_UINT32 ui32TruncateMode;
	FW_PERF_CONF eFirmwarePerf;
	RGX_ACTIVEPM_CONF eRGXActivePMConf;
	RGX_META_T1_CONF eUseMETAT1;
	RGX_RD_POWER_ISLAND_CONF eRGXRDPowerIslandConf;

#if defined(SUPPORT_GPUVIRT_VALIDATION)
	IMG_UINT32 aui32OSidMin[GPUVIRT_VALIDATION_NUM_REGIONS][GPUVIRT_VALIDATION_NUM_OS];
	IMG_UINT32 aui32OSidMax[GPUVIRT_VALIDATION_NUM_REGIONS][GPUVIRT_VALIDATION_NUM_OS];
#endif
	IMG_BOOL   bEnableTrustedDeviceAceConfig;
	IMG_UINT32 ui32FWContextSwitchCrossDM;
#if defined(SUPPORT_PHYSMEM_TEST) && !defined(INTEGRITY_OS) && !defined(__QNXNTO__)
	IMG_UINT32 ui32PhysMemTestPasses;
#endif
} RGX_SRVINIT_APPHINTS;


#if defined(SUPPORT_GPUVIRT_VALIDATION)
/*
 * Parses the dot('.') separated OSID regions on a string and stores the integer results
 * in an array. Numbers can be decimal or hex (starting with 0x) and there must be a . between each
 * (example: 1.2.3.4.5.6.7.8)
 */
static void _ParseOSidRegionString(IMG_CHAR *apszBuffer, IMG_UINT32 *pui32ApphintArray)
{
	IMG_UINT32 ui32OSid;
	IMG_CHAR *pui8StringParsingBase=apszBuffer;
	IMG_UINT32 ui32StringLength = OSStringLength(apszBuffer);

	/* Initialize all apphints to 0 */
	for (ui32OSid = 0; ui32OSid < GPUVIRT_VALIDATION_NUM_OS; ui32OSid++)
	{
		pui32ApphintArray[ui32OSid] = 0;
	}

	/* Parse the string. Even if it fails, apphints will have been initialized */
	for (ui32OSid = 0; ui32OSid < GPUVIRT_VALIDATION_NUM_OS; ui32OSid++)
	{
		IMG_UINT32 ui32Base=10;
		IMG_CHAR *pui8StringParsingNextDelimiter;

		/* Find the next character in the string that's not a ',' '.' or ' ' */
		while ((*pui8StringParsingBase == '.' ||
			    *pui8StringParsingBase == ',' ||
			    *pui8StringParsingBase == ' ') &&
			   pui8StringParsingBase - apszBuffer <= ui32StringLength)
		{
			pui8StringParsingBase++;
		}

		if (pui8StringParsingBase - apszBuffer > ui32StringLength)
		{
			PVR_DPF((PVR_DBG_ERROR, "Reached the end of the apphint string while trying to parse it.\nBuffer: %s, OSid: %d", pui8StringParsingBase, ui32OSid));
			return;
		}

		/* If the substring begins with "0x" move the pointer 2 bytes forward and set the base to 16 */
		if (*pui8StringParsingBase == '0' && *(pui8StringParsingBase+1) =='x')
		{
			ui32Base=16;
			pui8StringParsingBase+=2;
		}

		/* Find the next delimiter in the string or the end of the string itself if we're parsing the final number */
		pui8StringParsingNextDelimiter = pui8StringParsingBase;

		while (*pui8StringParsingNextDelimiter!='.' &&
			   *pui8StringParsingNextDelimiter!=',' &&
			   *pui8StringParsingNextDelimiter!=' ' &&
			   *pui8StringParsingNextDelimiter!='\0' &&
			   (pui8StringParsingNextDelimiter - apszBuffer <= ui32StringLength))
		{
			pui8StringParsingNextDelimiter++;
		}

		/*
		 * Each number is followed by a '.' except for the last one. If a string termination is found
		 * when not expected the functions returns
		 */

		if (*pui8StringParsingNextDelimiter=='\0' && ui32OSid < GPUVIRT_VALIDATION_NUM_OS - 1)
		{
			PVR_DPF((PVR_DBG_ERROR, "There was an error parsing the OSid Region Apphint Strings"));
			return;
		}

		/*replace the . with a string termination so that it can be properly parsed to an integer */
		*pui8StringParsingNextDelimiter = '\0';

		/* Parse the number. The fact that it is followed by '\0' means that the string parsing utility
		 * will finish there and not try to parse the rest */

		OSStringToUINT32(pui8StringParsingBase, ui32Base, &pui32ApphintArray[ui32OSid]);

		pui8StringParsingBase = pui8StringParsingNextDelimiter + 1;
	}
}

#endif
/*!
*******************************************************************************

 @Function      GetApphints

 @Description   Read init time apphints and initialise internal variables

 @Input         psHints : Pointer to apphints container

 @Return        void

******************************************************************************/
static INLINE void GetApphints(PVRSRV_RGXDEV_INFO *psDevInfo, RGX_SRVINIT_APPHINTS *psHints)
{
	void *pvParamState = SrvInitParamOpen();
	IMG_UINT32 ui32ParamTemp;
	IMG_BOOL bS7TopInfra = IMG_FALSE, bE42290 = IMG_FALSE, bTPUFiltermodeCtrl = IMG_FALSE;
	IMG_BOOL bE42606 = IMG_FALSE, bAXIACELite = IMG_FALSE;

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, S7_TOP_INFRASTRUCTURE))
	{
		bS7TopInfra = IMG_TRUE;
	}

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, TPU_FILTERING_MODE_CONTROL))
	{
		bTPUFiltermodeCtrl = IMG_TRUE;
	}

	if (RGX_IS_ERN_SUPPORTED(psDevInfo, 42290))
	{
		bE42290 = IMG_TRUE;
	}

	if (RGX_IS_ERN_SUPPORTED(psDevInfo, 42606))
	{
		bE42606 = IMG_TRUE;
	}

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, AXI_ACELITE))
	{
		bAXIACELite = IMG_TRUE;
	}

	/*
	 * NB AppHints initialised to a default value via SrvInitParamInit* macros above
	 */
	SrvInitParamGetUINT32(pvParamState,   DriverMode, psHints->ui32DriverMode);
	SrvInitParamGetBOOL(pvParamState,     DustRequestInject, psHints->bDustRequestInject);
	SrvInitParamGetBOOL(pvParamState,     EnableSignatureChecks, psHints->bEnableSignatureChecks);
	SrvInitParamGetUINT32(pvParamState,   SignatureChecksBufSize, psHints->ui32SignatureChecksBufSize);

#if defined(DEBUG)
	SrvInitParamGetBOOL(pvParamState,    AssertOutOfMemory, psHints->bAssertOnOutOfMem);
#endif
	SrvInitParamGetBOOL(pvParamState,    AssertOnHWRTrigger, psHints->bAssertOnHWRTrigger);
	SrvInitParamGetBOOL(pvParamState,    CheckMList, psHints->bCheckMlist);
	SrvInitParamGetBOOL(pvParamState,    DisableClockGating, psHints->bDisableClockGating);
	SrvInitParamGetBOOL(pvParamState,    DisableDMOverlap, psHints->bDisableDMOverlap);
	SrvInitParamGetBOOL(pvParamState,    DisableFEDLogging, psHints->bDisableFEDLogging);
	SrvInitParamGetUINT32(pvParamState,  EnableAPM, ui32ParamTemp);
	psHints->eRGXActivePMConf = ui32ParamTemp;
	SrvInitParamGetBOOL(pvParamState,    EnableCDMKillingRandMode, psHints->bEnableCDMKillRand);
	SrvInitParamGetUINT32(pvParamState,  EnableFWContextSwitch, psHints->ui32EnableFWContextSwitch);
	SrvInitParamGetUINT32(pvParamState,  VDMContextSwitchMode, psHints->ui32VDMContextSwitchMode);
	SrvInitParamGetBOOL(pvParamState,    EnableHWR, psHints->bEnableHWR);
	SrvInitParamGetUINT32(pvParamState,  EnableRDPowerIsland, ui32ParamTemp);
	psHints->eRGXRDPowerIslandConf = ui32ParamTemp;
	SrvInitParamGetUINT32(pvParamState,  FirmwarePerf, ui32ParamTemp);
	psHints->eFirmwarePerf = ui32ParamTemp;
	SrvInitParamGetUINT32(pvParamState,  FWContextSwitchProfile, psHints->ui32FWContextSwitchProfile);
	SrvInitParamGetBOOL(pvParamState,    HWPerfDisableCustomCounterFilter, psHints->bHWPerfDisableCustomCounterFilter);
	SrvInitParamGetUINT32(pvParamState,  HWPerfHostBufSizeInKB, psHints->ui32HWPerfHostBufSize);
	SrvInitParamGetUINT32(pvParamState,  HWPerfFWBufSizeInKB, psHints->ui32HWPerfFWBufSize);
#if defined(LINUX)
	/* name changes */
	{
		IMG_UINT64 ui64Tmp;
		SrvInitParamGetBOOL(pvParamState,    DisablePDumpPanic, psHints->bDisablePDP);
		SrvInitParamGetUINT64(pvParamState,  HWPerfFWFilter, ui64Tmp);
		psHints->ui32HWPerfFilter0 = (IMG_UINT32)(ui64Tmp & 0xffffffffllu);
		psHints->ui32HWPerfFilter1 = (IMG_UINT32)((ui64Tmp >> 32) & 0xffffffffllu);
	}
#else
	SrvInitParamUnreferenced(DisablePDumpPanic);
	SrvInitParamUnreferenced(HWPerfFWFilter);
	SrvInitParamUnreferenced(RGXBVNC);
#endif
	SrvInitParamGetUINT32(pvParamState, HWPerfHostFilter, psHints->ui32HWPerfHostFilter);
	SrvInitParamGetUINT32List(pvParamState, TimeCorrClock, psHints->ui32TimeCorrClock);
	SrvInitParamGetUINT32(pvParamState, HWRDebugDumpLimit, ui32ParamTemp);
	psHints->ui32HWRDebugDumpLimit = MIN(ui32ParamTemp, RGXFWIF_HWR_DEBUG_DUMP_ALL);

	if (bS7TopInfra)
	{
	#define RGX_CR_JONES_FIX_MT_ORDER_ISP_TE_CLRMSK	(0XFFFFFFCFU)
	#define RGX_CR_JONES_FIX_MT_ORDER_ISP_EN	(0X00000020U)
	#define RGX_CR_JONES_FIX_MT_ORDER_TE_EN		(0X00000010U)

		SrvInitParamGetUINT32(pvParamState,  JonesDisableMask, ui32ParamTemp);
		if (((ui32ParamTemp & ~RGX_CR_JONES_FIX_MT_ORDER_ISP_TE_CLRMSK) == RGX_CR_JONES_FIX_MT_ORDER_ISP_EN) ||
			((ui32ParamTemp & ~RGX_CR_JONES_FIX_MT_ORDER_ISP_TE_CLRMSK) == RGX_CR_JONES_FIX_MT_ORDER_TE_EN))
		{
			ui32ParamTemp |= (RGX_CR_JONES_FIX_MT_ORDER_TE_EN |
							  RGX_CR_JONES_FIX_MT_ORDER_ISP_EN);
			PVR_DPF((PVR_DBG_WARNING, "Tile reordering mode requires both TE and ISP enabled. Forcing JonesDisableMask = %d",
					ui32ParamTemp));
		}
		psHints->ui32JonesDisableMask = ui32ParamTemp;
	}

	if ((bE42290) && (bTPUFiltermodeCtrl))
	{
		SrvInitParamGetBOOL(pvParamState, NewFilteringMode, psHints->bFilteringMode);
	}

	if (bE42606)
	{
		SrvInitParamGetUINT32(pvParamState, TruncateMode, psHints->ui32TruncateMode);
	}
#if defined(EMULATOR)
	if (bAXIACELite)
	{
		SrvInitParamGetBOOL(pvParamState, EnableTrustedDeviceAceConfig, psHints->bEnableTrustedDeviceAceConfig);
	}
#endif

	SrvInitParamGetUINT32(pvParamState, UseMETAT1, ui32ParamTemp);
	psHints->eUseMETAT1 = ui32ParamTemp & RGXFWIF_INICFG_METAT1_MASK;

	SrvInitParamGetBOOL(pvParamState, ZeroFreelist, psHints->bZeroFreelist);

#if defined(LINUX)
	SrvInitParamGetUINT32(pvParamState, FWContextSwitchCrossDM, psHints->ui32FWContextSwitchCrossDM);
#else
	SrvInitParamUnreferenced(FWContextSwitchCrossDM);
#endif

#if defined(SUPPORT_PHYSMEM_TEST) && !defined(INTEGRITY_OS) && !defined(__QNXNTO__)
	SrvInitParamGetUINT32(pvParamState, PhysMemTestPasses, psHints->ui32PhysMemTestPasses);
#endif

#if defined(SUPPORT_VALIDATION)
	/* Apphints for TPU trilinear frac masking */
	SrvInitParamGetUINT32(pvParamState, TPUTrilinearFracMaskPDM, psHints->aui32TPUTrilinearFracMask[RGXFWIF_TPU_DM_PDM]);
	SrvInitParamGetUINT32(pvParamState, TPUTrilinearFracMaskVDM, psHints->aui32TPUTrilinearFracMask[RGXFWIF_TPU_DM_VDM]);
	SrvInitParamGetUINT32(pvParamState, TPUTrilinearFracMaskCDM, psHints->aui32TPUTrilinearFracMask[RGXFWIF_TPU_DM_CDM]);
	SrvInitParamGetUINT32(pvParamState, TPUTrilinearFracMaskTDM, psHints->aui32TPUTrilinearFracMask[RGXFWIF_TPU_DM_TDM]);
	SrvInitParamGetBOOL(pvParamState,   ValidateIrq, psHints->bValidateIrq);
	SrvInitParamGetUINT32(pvParamState, FBCDCVersionOverride, psHints->ui32FBCDCVersionOverride);
#endif

	/*
	 * FW logs apphints
	 */
	{
		IMG_UINT32 ui32LogType;
		IMG_BOOL bAnyLogGroupConfigured;

		SrvInitParamGetUINT32BitField(pvParamState, EnableLogGroup, ui32LogType);
		bAnyLogGroupConfigured = ui32LogType ? IMG_TRUE : IMG_FALSE;
		SrvInitParamGetUINT32List(pvParamState, FirmwareLogType, ui32ParamTemp);

		/* Defaulting to TRACE */
		ui32LogType |= RGXFWIF_LOG_TYPE_TRACE;

		if (ui32ParamTemp == 2 /* TRACE */)
		{
			if (!bAnyLogGroupConfigured)
			{
				/* No groups configured - defaulting to MAIN group */
				ui32LogType |= RGXFWIF_LOG_TYPE_GROUP_MAIN;
			}
		}
		else if (ui32ParamTemp == 1 /* TBI */)
		{
			if (!bAnyLogGroupConfigured)
			{
				/* No groups configured - defaulting to MAIN group */
				ui32LogType |= RGXFWIF_LOG_TYPE_GROUP_MAIN;
			}
			ui32LogType &= ~RGXFWIF_LOG_TYPE_TRACE;
		}
		else if (ui32ParamTemp == 0 /* NONE */)
		{
			/* "NONE" means "TRACE without any log groups enabled */
			ui32LogType = RGXFWIF_LOG_TYPE_TRACE;
		}

		/* MTK default FW debug flags */
#if !((IMG_1_11_OPTS) & IMG_1_11_OPT_DISABLE_FW_LOGGING_BY_DEFAULT)
		ui32LogType |= (RGXFWIF_LOG_TYPE_TRACE |
				RGXFWIF_LOG_TYPE_GROUP_MAIN |
				RGXFWIF_LOG_TYPE_GROUP_PM |
				RGXFWIF_LOG_TYPE_GROUP_POW |
				RGXFWIF_LOG_TYPE_GROUP_HWR);
#endif

		psHints->ui32LogType = ui32LogType;
	}

#if defined(SUPPORT_GPUVIRT_VALIDATION)
	/*
	 * GPU virtualisation validation apphints
	 */
	{
		IMG_CHAR pszOSidRegionBuffer[GPUVIRT_VALIDATION_MAX_STRING_LENGTH];

		SrvInitParamGetSTRING(pvParamState, OSidRegion0Min, pszOSidRegionBuffer, GPUVIRT_VALIDATION_MAX_STRING_LENGTH);
		_ParseOSidRegionString(pszOSidRegionBuffer, psHints->aui32OSidMin[0]);

		SrvInitParamGetSTRING(pvParamState, OSidRegion0Max, pszOSidRegionBuffer, GPUVIRT_VALIDATION_MAX_STRING_LENGTH);
		_ParseOSidRegionString(pszOSidRegionBuffer, psHints->aui32OSidMax[0]);

		SrvInitParamGetSTRING(pvParamState, OSidRegion1Min, pszOSidRegionBuffer, GPUVIRT_VALIDATION_MAX_STRING_LENGTH);
		_ParseOSidRegionString(pszOSidRegionBuffer, psHints->aui32OSidMin[1]);

		SrvInitParamGetSTRING(pvParamState, OSidRegion1Max, pszOSidRegionBuffer, GPUVIRT_VALIDATION_MAX_STRING_LENGTH);
		_ParseOSidRegionString(pszOSidRegionBuffer, psHints->aui32OSidMax[1]);
	}
#else
#if !defined(LINUX)
	SrvInitParamUnreferenced(OSidRegion0Min);
	SrvInitParamUnreferenced(OSidRegion0Max);
	SrvInitParamUnreferenced(OSidRegion1Min);
	SrvInitParamUnreferenced(OSidRegion1Max);
#endif /* !defined(LINUX) */
#endif /* defined(SUPPORT_GPUVIRT_VALIDATION) */


	SrvInitParamClose(pvParamState);
}


/*!
*******************************************************************************

 @Function      GetFWConfigFlags

 @Description   Initialise and return FW config flags

 @Input         psHints            : Apphints container
 @Input         pui32FWConfigFlags : Pointer to config flags

 @Return        void

******************************************************************************/
static INLINE void GetFWConfigFlags(PVRSRV_DEVICE_NODE *psDeviceNode,
                                    RGX_SRVINIT_APPHINTS *psHints,
                                    IMG_UINT32 *pui32FWConfigFlags,
                                    IMG_UINT32 *pui32FWConfigFlagsExt)
{
	IMG_UINT32 ui32FWConfigFlags = 0;

#if defined(DEBUG)
	ui32FWConfigFlags |= psHints->bAssertOnOutOfMem ? RGXFWIF_INICFG_ASSERT_ON_OUTOFMEMORY : 0;
#endif
	ui32FWConfigFlags |= psHints->bAssertOnHWRTrigger ? RGXFWIF_INICFG_ASSERT_ON_HWR_TRIGGER : 0;
	ui32FWConfigFlags |= psHints->bCheckMlist ? RGXFWIF_INICFG_CHECK_MLIST_EN : 0;
	ui32FWConfigFlags |= psHints->bDisableClockGating ? RGXFWIF_INICFG_DISABLE_CLKGATING_EN : 0;
	ui32FWConfigFlags |= psHints->bDisableDMOverlap ? RGXFWIF_INICFG_DISABLE_DM_OVERLAP : 0;
	ui32FWConfigFlags |= psHints->bDisablePDP ? RGXFWIF_INICFG_DISABLE_PDP_EN : 0;
	ui32FWConfigFlags |= psHints->bEnableCDMKillRand ? RGXFWIF_INICFG_CDM_KILL_MODE_RAND_EN : 0;
	ui32FWConfigFlags |= (psHints->ui32HWPerfFilter0 != 0 || psHints->ui32HWPerfFilter1 != 0) ? RGXFWIF_INICFG_HWPERF_EN : 0;
#if !defined(NO_HARDWARE)
	ui32FWConfigFlags |= psHints->bEnableHWR ? RGXFWIF_INICFG_HWR_EN : 0;
#endif
	ui32FWConfigFlags |= psHints->bHWPerfDisableCustomCounterFilter ? RGXFWIF_INICFG_HWP_DISABLE_FILTER : 0;
	ui32FWConfigFlags |= (psHints->eFirmwarePerf == FW_PERF_CONF_CUSTOM_TIMER) ? RGXFWIF_INICFG_CUSTOM_PERF_TIMER_EN : 0;
	ui32FWConfigFlags |= (psHints->eFirmwarePerf == FW_PERF_CONF_POLLS) ? RGXFWIF_INICFG_POLL_COUNTERS_EN : 0;
	ui32FWConfigFlags |= psHints->eUseMETAT1 << RGXFWIF_INICFG_METAT1_SHIFT;
	ui32FWConfigFlags |= psHints->ui32EnableFWContextSwitch & ~RGXFWIF_INICFG_CTXSWITCH_CLRMSK;
	ui32FWConfigFlags |= (psHints->ui32VDMContextSwitchMode << RGXFWIF_INICFG_VDM_CTX_STORE_MODE_SHIFT) & RGXFWIF_INICFG_VDM_CTX_STORE_MODE_MASK;

	ui32FWConfigFlags |= (psHints->ui32FWContextSwitchProfile << RGXFWIF_INICFG_CTXSWITCH_PROFILE_SHIFT) & RGXFWIF_INICFG_CTXSWITCH_PROFILE_MASK;

	*pui32FWConfigFlags = ui32FWConfigFlags;

	*pui32FWConfigFlagsExt = psHints->ui32FWContextSwitchCrossDM;
#if defined(SUPPORT_VALIDATION)
#if defined(NO_HARDWARE) && defined(PDUMP)
	*pui32FWConfigFlagsExt |= psHints->bValidateIrq ? RGXFWIF_INICFG_EXT_VALIDATE_IRQ : 0;
#endif

	if (psHints->ui32FBCDCVersionOverride > 0)
	{
		*pui32FWConfigFlagsExt |= (psHints->ui32FBCDCVersionOverride == 2) ? RGXFWIF_INICFG_EXT_FBCDC_V3_1_EN : 0;
	}
	else
#endif
	{
		*pui32FWConfigFlagsExt |= psDeviceNode->pfnHasFBCDCVersion31(psDeviceNode) ? RGXFWIF_INICFG_EXT_FBCDC_V3_1_EN : 0;
	}

}


/*!
*******************************************************************************

 @Function      GetFilterFlags

 @Description   Initialise and return filter flags

 @Input         psHints : Apphints container

 @Return        IMG_UINT32 : Filter flags

******************************************************************************/
static INLINE IMG_UINT32 GetFilterFlags(RGX_SRVINIT_APPHINTS *psHints)
{
	IMG_UINT32 ui32FilterFlags = 0;

	ui32FilterFlags |= psHints->bFilteringMode ? RGXFWIF_FILTCFG_NEW_FILTER_MODE : 0;
	if (psHints->ui32TruncateMode == 2)
	{
		ui32FilterFlags |= RGXFWIF_FILTCFG_TRUNCATE_INT;
	}
	else if (psHints->ui32TruncateMode == 3)
	{
		ui32FilterFlags |= RGXFWIF_FILTCFG_TRUNCATE_HALF;
	}

	return ui32FilterFlags;
}


/*!
*******************************************************************************

 @Function      GetDeviceFlags

 @Description   Initialise and return device flags

 @Input         psHints          : Apphints container
 @Input         pui32DeviceFlags : Pointer to device flags

 @Return        void

******************************************************************************/
static INLINE void GetDeviceFlags(RGX_SRVINIT_APPHINTS *psHints,
                                  IMG_UINT32 *pui32DeviceFlags)
{
	IMG_UINT32 ui32DeviceFlags = 0;

	ui32DeviceFlags |= psHints->bDustRequestInject? RGXKM_DEVICE_STATE_DUST_REQUEST_INJECT_EN : 0;
	ui32DeviceFlags |= psHints->bZeroFreelist ? RGXKM_DEVICE_STATE_ZERO_FREELIST : 0;
	ui32DeviceFlags |= psHints->bDisableFEDLogging ? RGXKM_DEVICE_STATE_DISABLE_DW_LOGGING_EN : 0;

	*pui32DeviceFlags = ui32DeviceFlags;
}

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
/*!
*******************************************************************************

 @Function      RGXTDProcessFWImage

 @Description   Fetch and send data used by the trusted device to complete
                the FW image setup

 @Input         psDeviceNode : Device node
 @Input         psRGXFW      : Firmware blob
 @Input         puFWParams   : Parameters used by the FW at boot time

 @Return        PVRSRV_ERROR
******************************************************************************/
static PVRSRV_ERROR RGXTDProcessFWImage(PVRSRV_DEVICE_NODE *psDeviceNode,
                                        struct RGXFW *psRGXFW,
                                        RGX_FW_BOOT_PARAMS *puFWParams)
{
	PVRSRV_DEVICE_CONFIG *psDevConfig = psDeviceNode->psDevConfig;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	PVRSRV_TD_FW_PARAMS sTDFWParams;
	RGX_LAYER_PARAMS sLayerParams;
	PVRSRV_ERROR eError;

	if (psDevConfig->pfnTDSendFWImage == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXTDProcessFWImage: TDSendFWImage not implemented!"));
		return PVRSRV_ERROR_NOT_IMPLEMENTED;
	}

	sLayerParams.psDevInfo = psDevInfo;

	sTDFWParams.pvFirmware       = RGXFirmwareData(psRGXFW);
	sTDFWParams.ui32FirmwareSize = RGXFirmwareSize(psRGXFW);

	if (!RGX_DEVICE_HAS_FEATURE(&sLayerParams, MIPS))
	{
		sTDFWParams.uFWP.sMeta.sFWCodeDevVAddr        = puFWParams->sMeta.sFWCodeDevVAddr;
		sTDFWParams.uFWP.sMeta.sFWDataDevVAddr        = puFWParams->sMeta.sFWDataDevVAddr;
		sTDFWParams.uFWP.sMeta.sFWCorememCodeDevVAddr = puFWParams->sMeta.sFWCorememCodeDevVAddr;
		sTDFWParams.uFWP.sMeta.sFWCorememCodeFWAddr   = puFWParams->sMeta.sFWCorememCodeFWAddr;
		sTDFWParams.uFWP.sMeta.uiFWCorememCodeSize    = puFWParams->sMeta.uiFWCorememCodeSize;
		sTDFWParams.uFWP.sMeta.sFWCorememDataDevVAddr = puFWParams->sMeta.sFWCorememDataDevVAddr;
		sTDFWParams.uFWP.sMeta.sFWCorememDataFWAddr   = puFWParams->sMeta.sFWCorememDataFWAddr;
		sTDFWParams.uFWP.sMeta.ui32NumThreads         = puFWParams->sMeta.ui32NumThreads;
		sTDFWParams.uFWP.sMeta.ui32MainThreadID       = puFWParams->sMeta.ui32MainThreadID;
	}
	else
	{
		sTDFWParams.uFWP.sMips.sGPURegAddr            = puFWParams->sMips.sGPURegAddr;
		sTDFWParams.uFWP.sMips.sFWPageTableAddr       = puFWParams->sMips.sFWPageTableAddr;
		sTDFWParams.uFWP.sMips.sFWStackAddr           = puFWParams->sMips.sFWStackAddr;
	}

	eError = psDevConfig->pfnTDSendFWImage(psDevConfig->hSysData, &sTDFWParams);

	return eError;
}
#endif

/*!
*******************************************************************************

 @Function      RGXAcquireMipsBootldrData

 @Description   Acquire MIPS bootloader data parameters

 @Input         psDeviceNode : Device node
 @Input         puFWParams   : FW boot parameters

 @Return        PVRSRV_ERROR

******************************************************************************/
static PVRSRV_ERROR RGXAcquireMipsBootldrData(PVRSRV_DEVICE_NODE *psDeviceNode,
                                              RGX_FW_BOOT_PARAMS *puFWParams)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = (PVRSRV_RGXDEV_INFO*) psDeviceNode->pvDevice;
	PVRSRV_ERROR eError;
	IMG_BOOL bValid;

	/* Rogue Registers physical address */
#if defined(SUPPORT_ALT_REGBASE)
	puFWParams->sMips.sGPURegAddr = psDeviceNode->psDevConfig->sAltRegsGpuPBase;
#else
	PhysHeapCpuPAddrToDevPAddr(psDevInfo->psDeviceNode->apsPhysHeap[PVRSRV_DEVICE_PHYS_HEAP_GPU_LOCAL],
	                           1,
	                           &puFWParams->sMips.sGPURegAddr,
	                           &(psDeviceNode->psDevConfig->sRegsCpuPBase));
#endif

	/* MIPS Page Table physical address */
	MMU_AcquireBaseAddr(psDevInfo->psKernelMMUCtx, &puFWParams->sMips.sFWPageTableAddr);

	/* MIPS Stack Pointer Physical Address */
	eError = RGXGetPhyAddr(psDevInfo->psRGXFWDataMemDesc->psImport->hPMR,
	                       &puFWParams->sMips.sFWStackAddr,
	                       RGXGetFWImageSectionOffset(NULL, MIPS_STACK),
	                       OSGetPageShift(),
	                       1,
	                       &bValid);

	return eError;
}

/*!
*******************************************************************************

 @Function      InitFirmware

 @Description   Allocate, initialise and pdump Firmware code and data memory

 @Input         psDeviceNode : Device Node
 @Input         psHints      : Apphints

 @Return        PVRSRV_ERROR

******************************************************************************/
static PVRSRV_ERROR InitFirmware(PVRSRV_DEVICE_NODE *psDeviceNode,
                                 RGX_SRVINIT_APPHINTS *psHints)
{
	struct RGXFW      *psRGXFW = NULL;
	const IMG_BYTE    *pbRGXFirmware = NULL;

	/* FW code memory */
	IMG_DEVMEM_SIZE_T uiFWCodeAllocSize;
	void              *pvFWCodeHostAddr;

	/* FW data memory */
	IMG_DEVMEM_SIZE_T uiFWDataAllocSize;
	void              *pvFWDataHostAddr;

	/* FW coremem code memory */
	IMG_DEVMEM_SIZE_T uiFWCorememCodeAllocSize;
	void              *pvFWCorememCodeHostAddr = NULL;

	/* FW coremem data memory */
	IMG_DEVMEM_SIZE_T uiFWCorememDataAllocSize;
	void              *pvFWCorememDataHostAddr = NULL;

	RGX_FW_BOOT_PARAMS uFWParams;
	RGX_LAYER_PARAMS sLayerParams;
	PVRSRV_ERROR eError;
	PVRSRV_RGXDEV_INFO *psDevInfo = (PVRSRV_RGXDEV_INFO *)psDeviceNode->pvDevice;

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	IMG_BOOL bUseSecureFWData = RGX_IS_FEATURE_SUPPORTED(psDevInfo, MIPS) &&
	                            RGX_GET_FEATURE_VALUE(psDevInfo, PHYS_BUS_WIDTH) > 32;
#endif

	/*
	 * Get pointer to Firmware image
	 */
	pbRGXFirmware = RGXLoadAndGetFWData(psDeviceNode, &psRGXFW);
	if (!pbRGXFirmware)
	{
		/* Error or confirmation message generated in RGXLoadAndGetFWData */
		eError = PVRSRV_ERROR_INIT_FAILURE;
		goto cleanup_initfw;
	}

	sLayerParams.psDevInfo = psDevInfo;

	/*
	 * Allocate Firmware memory
	 */

	eError = RGXGetFWImageAllocSize(&sLayerParams,
	                                pbRGXFirmware,
	                                RGXFirmwareSize(psRGXFW),
	                                &uiFWCodeAllocSize,
	                                &uiFWDataAllocSize,
	                                &uiFWCorememCodeAllocSize,
	                                &uiFWCorememDataAllocSize);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "%s: RGXGetFWImageAllocSize failed", __func__));
		goto cleanup_initfw;
	}

	psDevInfo->ui32FWCorememCodeSizeInBytes = uiFWCorememCodeAllocSize;
	psDevInfo->ui32FWCodeSizeInBytes = uiFWCodeAllocSize;

#if defined(SUPPORT_TRUSTED_DEVICE)
	/* Disable META core memory allocation unless the META DMA is available */
	if (!RGX_DEVICE_HAS_FEATURE(&sLayerParams, META_DMA))
	{
		uiFWCorememCodeAllocSize = 0;
		uiFWCorememDataAllocSize = 0;
	}
#endif
	eError = RGXInitAllocFWImgMem(psDeviceNode,
	                              uiFWCodeAllocSize,
	                              uiFWDataAllocSize,
	                              uiFWCorememCodeAllocSize,
	                              uiFWCorememDataAllocSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "%s: PVRSRVRGXInitAllocFWImgMemKM failed (%d)",
				 __func__, eError));
		goto cleanup_initfw;
	}

	/*
	 * Acquire pointers to Firmware allocations
	 */

#if !defined(SUPPORT_TRUSTED_DEVICE) || defined(NO_HARDWARE)
	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWCodeMemDesc, &pvFWCodeHostAddr);
	PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", cleanup_initfw);

#else
	/* We can't get a pointer to a secure FW allocation from within the DDK */
	pvFWCodeHostAddr = NULL;
#endif

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	if (bUseSecureFWData)
	{
		/* We can't get a pointer to a secure FW allocation from within the DDK */
		pvFWDataHostAddr = NULL;
	}
	else
#endif
	{
		eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWDataMemDesc, &pvFWDataHostAddr);
		PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", release_code);
	}

#if !defined(SUPPORT_TRUSTED_DEVICE) || defined(NO_HARDWARE)
	if (uiFWCorememCodeAllocSize)
	{
		eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWCorememCodeMemDesc, &pvFWCorememCodeHostAddr);
		PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", release_data);
	}
#else
	/* We can't get a pointer to a secure FW allocation from within the DDK */
	pvFWCorememCodeHostAddr = NULL;
#endif

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	if (bUseSecureFWData)
	{
		pvFWCorememDataHostAddr = NULL;
	}
	else
#endif
	if (uiFWCorememDataAllocSize)
	{
		eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfCorememDataStoreMemDesc, &pvFWCorememDataHostAddr);
		PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", release_corememcode);
	}

	/*
	 * Prepare FW boot parameters
	 */

	if (RGX_DEVICE_HAS_FEATURE(&sLayerParams, MIPS))
	{
		eError = RGXAcquireMipsBootldrData(psDeviceNode, &uFWParams);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					 "%s: RGXAcquireMipsBootldrData failed (%d)",
					 __func__, eError));
			goto release_fw_allocations;
		}
	}
	else
	{
		uFWParams.sMeta.sFWCodeDevVAddr = psDevInfo->sFWCodeDevVAddrBase;
		uFWParams.sMeta.sFWDataDevVAddr = psDevInfo->sFWDataDevVAddrBase;
		uFWParams.sMeta.sFWCorememCodeDevVAddr = psDevInfo->sFWCorememCodeDevVAddrBase;
		uFWParams.sMeta.sFWCorememCodeFWAddr = psDevInfo->sFWCorememCodeFWAddr;
		uFWParams.sMeta.uiFWCorememCodeSize = uiFWCorememCodeAllocSize;
		uFWParams.sMeta.sFWCorememDataDevVAddr = psDevInfo->sFWCorememDataStoreDevVAddrBase;
		uFWParams.sMeta.sFWCorememDataFWAddr = psDevInfo->sFWCorememDataStoreFWAddr;
#if defined(RGXFW_META_SUPPORT_2ND_THREAD)
		uFWParams.sMeta.ui32NumThreads = 2,
#else
		uFWParams.sMeta.ui32NumThreads = psHints->eUseMETAT1 == RGX_META_T1_OFF ? 1 : 2;
#endif
		uFWParams.sMeta.ui32MainThreadID = psHints->eUseMETAT1 == RGX_META_T1_MAIN ? 1 : 0;
	}


	/*
	 * Process the Firmware image and setup code and data segments.
	 *
	 * When the trusted device is enabled and the FW code lives
	 * in secure memory we will only setup the data segments here,
	 * while the code segments will be loaded to secure memory
	 * by the trusted device.
	 */
	eError = RGXProcessFWImage(&sLayerParams,
							   pbRGXFirmware,
							   pvFWCodeHostAddr,
							   pvFWDataHostAddr,
							   pvFWCorememCodeHostAddr,
							   pvFWCorememDataHostAddr,
							   &uFWParams);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "%s: RGXProcessFWImage failed (%d)",
				 __func__, eError));
		goto release_fw_allocations;
	}

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	RGXTDProcessFWImage(psDeviceNode, psRGXFW, &uFWParams);
#endif


	/*
	 * PDump Firmware allocations
	 */

#if !defined(SUPPORT_TRUSTED_DEVICE) || defined(NO_HARDWARE)
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Dump firmware code image");
	DevmemPDumpLoadMem(psDevInfo->psRGXFWCodeMemDesc,
	                   0,
	                   uiFWCodeAllocSize,
	                   PDUMP_FLAGS_CONTINUOUS);
#endif

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	if (!bUseSecureFWData)
#endif
	{
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Dump firmware data image");
		DevmemPDumpLoadMem(psDevInfo->psRGXFWDataMemDesc,
		                   0,
		                   uiFWDataAllocSize,
		                   PDUMP_FLAGS_CONTINUOUS);
	}

#if !defined(SUPPORT_TRUSTED_DEVICE) || defined(NO_HARDWARE)
	if (uiFWCorememCodeAllocSize)
	{
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Dump firmware coremem code image");
		DevmemPDumpLoadMem(psDevInfo->psRGXFWCorememCodeMemDesc,
						   0,
						   uiFWCorememCodeAllocSize,
						   PDUMP_FLAGS_CONTINUOUS);
	}
#endif

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	if (!bUseSecureFWData && uiFWCorememDataAllocSize)
#else
	if (uiFWCorememDataAllocSize)
#endif
	{
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Dump firmware coremem data store image");
		DevmemPDumpLoadMem(psDevInfo->psRGXFWIfCorememDataStoreMemDesc,
						   0,
						   uiFWCorememDataAllocSize,
						   PDUMP_FLAGS_CONTINUOUS);
	}

	/*
	 * Release Firmware allocations and clean up
	 */

release_fw_allocations:
#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	if (!bUseSecureFWData && uiFWCorememDataAllocSize)
#else
	if (uiFWCorememDataAllocSize)
#endif
	{
		DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfCorememDataStoreMemDesc);
	}
release_corememcode:
#if !defined(SUPPORT_TRUSTED_DEVICE) || defined(NO_HARDWARE)
	if (uiFWCorememCodeAllocSize)
	{
		DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWCorememCodeMemDesc);
	}
#endif

#if !defined(SUPPORT_TRUSTED_DEVICE) || defined(NO_HARDWARE)
release_data:
#endif
#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	if (!bUseSecureFWData)
#endif
	{
		DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWDataMemDesc);
	}

release_code:
#if !defined(SUPPORT_TRUSTED_DEVICE) || defined(NO_HARDWARE)
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWCodeMemDesc);
#endif
cleanup_initfw:
	if (psRGXFW != NULL)
	{
		RGXUnloadFirmware(psRGXFW);
	}

	return eError;
}


#if defined(PDUMP)
/*!
*******************************************************************************

 @Function      InitialiseHWPerfCounters

 @Description   Initialisation of hardware performance counters and dumping
                them out to pdump, so that they can be modified at a later
                point.

 @Input         pvDevice
 @Input         psHWPerfDataMemDesc
 @Input         psHWPerfInitDataInt

 @Return        void

******************************************************************************/

static void InitialiseHWPerfCounters(void *pvDevice, DEVMEM_MEMDESC *psHWPerfDataMemDesc, RGXFWIF_HWPERF_CTL *psHWPerfInitDataInt)
{
	RGXFWIF_HWPERF_CTL_BLK *psHWPerfInitBlkData;
	IMG_UINT32 ui32CntBlkModelLen;
	const RGXFW_HWPERF_CNTBLK_TYPE_MODEL *asCntBlkTypeModel;
	const RGXFW_HWPERF_CNTBLK_TYPE_MODEL* psBlkTypeDesc;
	IMG_UINT32 ui32BlockID, ui32BlkCfgIdx, ui32CounterIdx;
	RGX_HWPERF_CNTBLK_RT_INFO sCntBlkRtInfo;

	ui32CntBlkModelLen = RGXGetHWPerfBlockConfig(&asCntBlkTypeModel);
	for (ui32BlkCfgIdx = 0; ui32BlkCfgIdx < ui32CntBlkModelLen; ui32BlkCfgIdx++)
	{
		/* Exit early if this core does not have any of these counter blocks
		 * due to core type/BVNC features.... */
		psBlkTypeDesc = &asCntBlkTypeModel[ui32BlkCfgIdx];
		if (psBlkTypeDesc->pfnIsBlkPresent(psBlkTypeDesc, pvDevice, &sCntBlkRtInfo) == IMG_FALSE)
		{
			continue;
		}

		/* Program all counters in one block so those already on may
		 * be configured off and vice-a-versa. */
		for (ui32BlockID = psBlkTypeDesc->uiCntBlkIdBase;
					 ui32BlockID < psBlkTypeDesc->uiCntBlkIdBase+sCntBlkRtInfo.uiNumUnits;
					 ui32BlockID++)
		{

			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Unit %d Block : %s", ui32BlockID-psBlkTypeDesc->uiCntBlkIdBase, psBlkTypeDesc->pszBlockNameComment);
			/* Get the block configure store to update from the global store of
			 * block configuration. This is used to remember the configuration
			 * between configurations and core power on in APM */
			psHWPerfInitBlkData = rgxfw_hwperf_get_block_ctl(ui32BlockID, psHWPerfInitDataInt);
			/* Assert to check for HWPerf block mis-configuration */
			PVR_ASSERT(psHWPerfInitBlkData);

			psHWPerfInitBlkData->bValid = IMG_TRUE;
			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "bValid: This specifies if the layout block is valid for the given BVNC.");
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->bValid) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->bValid,
							PDUMP_FLAGS_CONTINUOUS);

			psHWPerfInitBlkData->bEnabled = IMG_FALSE;
			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "bEnabled: Set to 0x1 if the block needs to be enabled during playback.");
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->bEnabled) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->bEnabled,
							PDUMP_FLAGS_CONTINUOUS);

			psHWPerfInitBlkData->eBlockID = ui32BlockID;
			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "eBlockID: The Block ID for the layout block. See RGX_HWPERF_CNTBLK_ID for further information.");
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->eBlockID) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->eBlockID,
							PDUMP_FLAGS_CONTINUOUS);

			psHWPerfInitBlkData->uiCounterMask = 0x00;
			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "uiCounterMask: Bitmask for selecting the counters that need to be configured. (Bit 0 - counter0, bit 1 - counter1 and so on.");
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->uiCounterMask) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->uiCounterMask,
							PDUMP_FLAGS_CONTINUOUS);

			for (ui32CounterIdx = RGX_CNTBLK_COUNTER0_ID; ui32CounterIdx < psBlkTypeDesc->uiNumCounters; ui32CounterIdx++)
			{
				psHWPerfInitBlkData->aui64CounterCfg[ui32CounterIdx] = IMG_UINT64_C(0x0000000000000000);

				PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "%s_COUNTER_%d", psBlkTypeDesc->pszBlockNameComment,ui32CounterIdx);
				DevmemPDumpLoadMemValue64(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->aui64CounterCfg[ui32CounterIdx]) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->aui64CounterCfg[ui32CounterIdx],
							PDUMP_FLAGS_CONTINUOUS);

			}
		}
	}
}
/*!
*******************************************************************************

 @Function      InitialiseCustomCounters

 @Description   Initialisation of custom counters and dumping them out to
                pdump, so that they can be modified at a later point.

 @Input         psHWPerfDataMemDesc

 @Return        void

******************************************************************************/

static void InitialiseCustomCounters(DEVMEM_MEMDESC *psHWPerfDataMemDesc)
{
	IMG_UINT32 ui32CustomBlock, ui32CounterID;

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "ui32SelectedCountersBlockMask - The Bitmask of the custom counters that are to be selected");
	DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
						offsetof(RGXFWIF_HWPERF_CTL, ui32SelectedCountersBlockMask),
						0,
						PDUMP_FLAGS_CONTINUOUS);

	for (ui32CustomBlock = 0; ui32CustomBlock < RGX_HWPERF_MAX_CUSTOM_BLKS; ui32CustomBlock++)
	{
		/*
		 * Some compilers cannot cope with the use of offsetof() below - the specific problem being the use of
		 * a non-const variable in the expression, which it needs to be const. Typical compiler error produced is
		 * "expression must have a constant value".
		 */
		const IMG_DEVMEM_OFFSET_T uiOffsetOfCustomBlockSelectedCounters
		= (IMG_DEVMEM_OFFSET_T)(uintptr_t)&(((RGXFWIF_HWPERF_CTL *)0)->SelCntr[ui32CustomBlock].ui32NumSelectedCounters);

		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "ui32NumSelectedCounters - The Number of counters selected for this Custom Block: %d",ui32CustomBlock );
		DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
					uiOffsetOfCustomBlockSelectedCounters,
					0,
					PDUMP_FLAGS_CONTINUOUS);

		for (ui32CounterID = 0; ui32CounterID < RGX_HWPERF_MAX_CUSTOM_CNTRS; ui32CounterID++ )
		{
			const IMG_DEVMEM_OFFSET_T uiOffsetOfCustomBlockSelectedCounterIDs
			= (IMG_DEVMEM_OFFSET_T)(uintptr_t)&(((RGXFWIF_HWPERF_CTL *)0)->SelCntr[ui32CustomBlock].aui32SelectedCountersIDs[ui32CounterID]);

			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "CUSTOMBLK_%d_COUNTERID_%d",ui32CustomBlock, ui32CounterID);
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
					uiOffsetOfCustomBlockSelectedCounterIDs,
					0,
					PDUMP_FLAGS_CONTINUOUS);
		}
	}
}

/*!
*******************************************************************************

 @Function      InitialiseAllCounters

 @Description   Initialise HWPerf and custom counters

 @Input         psDeviceNode : Device Node

 @Return        PVRSRV_ERROR

******************************************************************************/
static PVRSRV_ERROR InitialiseAllCounters(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = (PVRSRV_RGXDEV_INFO *)psDeviceNode->pvDevice;
	RGXFWIF_HWPERF_CTL *psHWPerfInitData;
	PVRSRV_ERROR eError;

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfHWPerfCountersMemDesc, (void **)&psHWPerfInitData);
	PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", failHWPerfCountersMemDescAqCpuVirt);

	InitialiseHWPerfCounters(psDevInfo, psDevInfo->psRGXFWIfHWPerfCountersMemDesc, psHWPerfInitData);
	InitialiseCustomCounters(psDevInfo->psRGXFWIfHWPerfCountersMemDesc);

failHWPerfCountersMemDescAqCpuVirt:
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfHWPerfCountersMemDesc);

	return eError;
}
#endif /* PDUMP */

/*
 * _ParseHTBAppHints:
 *
 * Generate necessary references to the globally visible AppHints which are
 * declared in the above #include "km_apphint_defs.h"
 * Without these local references some compiler tool-chains will treat
 * unreferenced declarations as fatal errors. This function duplicates the
 * HTB_specific apphint references which are made in htbserver.c:HTBInit()
 * However, it makes absolutely *NO* use of these hints.
 */
static void
_ParseHTBAppHints(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	void *pvParamState = NULL;
	IMG_UINT32 ui32LogType;
	IMG_BOOL bAnyLogGroupConfigured;
	IMG_UINT32 ui32BufferSize;
	IMG_UINT32 ui32OpMode;

	/* Services initialisation parameters */
	pvParamState = SrvInitParamOpen();
	if (pvParamState == NULL)
		return;

	SrvInitParamGetUINT32BitField(pvParamState, EnableHTBLogGroup, ui32LogType);
	bAnyLogGroupConfigured = ui32LogType ? IMG_TRUE : IMG_FALSE;
	SrvInitParamGetUINT32List(pvParamState, HTBOperationMode, ui32OpMode);
	SrvInitParamGetUINT32(pvParamState, HTBufferSizeInKB, ui32BufferSize);

	SrvInitParamClose(pvParamState);
}

/*!
*******************************************************************************

 @Function      RGXInit

 @Description   RGX Initialisation

 @Input         psDeviceNode

 @Return        PVRSRV_ERROR

******************************************************************************/
PVRSRV_ERROR RGXInit(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_ERROR eError;

	/* Services initialisation parameters */
	RGX_SRVINIT_APPHINTS sApphints = {0};
	IMG_UINT32 ui32FWConfigFlags, ui32FWConfigFlagsExt;
	IMG_UINT32 ui32DeviceFlags;

	PVRSRV_RGXDEV_INFO *psDevInfo = (PVRSRV_RGXDEV_INFO *)psDeviceNode->pvDevice;

	/* Services initialisation parameters */
	_ParseHTBAppHints(psDeviceNode);
	GetApphints(psDevInfo, &sApphints);
	GetDeviceFlags(&sApphints, &ui32DeviceFlags);

#if defined(SUPPORT_GPUVIRT_VALIDATION)
	RGXVirtPopulateLMASubArenas(psDeviceNode, sApphints.aui32OSidMin, sApphints.aui32OSidMax, sApphints.bEnableTrustedDeviceAceConfig);
#endif

	eError = RGXInitCreateFWKernelMemoryContext(psDeviceNode);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to create FW kernel memory context (%u)",
		         __func__, eError));
		goto cleanup;
	}

	if (!PVRSRV_VZ_MODE_IS(DRIVER_MODE_GUEST))
	{
		eError = InitFirmware(psDeviceNode, &sApphints);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					 "%s: InitFirmware failed (%d)",
					 __func__, eError));
			goto cleanup;
		}
	}


	/*
	 * Setup Firmware initialisation data
	 */

	GetFWConfigFlags(psDeviceNode, &sApphints, &ui32FWConfigFlags, &ui32FWConfigFlagsExt);

	eError = RGXInitFirmware(psDeviceNode,
	                         sApphints.bEnableSignatureChecks,
	                         sApphints.ui32SignatureChecksBufSize,
	                         sApphints.ui32HWPerfFWBufSize,
	                         (IMG_UINT64)sApphints.ui32HWPerfFilter0 |
	                         ((IMG_UINT64)sApphints.ui32HWPerfFilter1 << 32),
	                         0,
	                         NULL,
	                         ui32FWConfigFlags,
	                         sApphints.ui32LogType,
	                         GetFilterFlags(&sApphints),
	                         sApphints.ui32JonesDisableMask,
	                         sApphints.ui32HWRDebugDumpLimit,
	                         sizeof(RGXFWIF_HWPERF_CTL),
#if defined(SUPPORT_VALIDATION)
	                         &sApphints.aui32TPUTrilinearFracMask[0],
#else
	                         NULL,
#endif
	                         sApphints.eRGXRDPowerIslandConf,
	                         sApphints.eFirmwarePerf,
	                         ui32FWConfigFlagsExt);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "%s: PVRSRVRGXInitFirmware failed (%d)",
				 __func__, eError));
		goto cleanup;
	}

#if defined(PDUMP)
	eError = InitialiseAllCounters(psDeviceNode);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "%s: InitialiseAllCounters failed (%d)",
				 __func__, eError));
		goto cleanup;
	}
#endif

	/*
	 * Perform second stage of RGX initialisation
	 */
	eError = RGXInitDevPart2(psDeviceNode,
	                         ui32DeviceFlags,
	                         sApphints.ui32HWPerfHostBufSize,
	                         sApphints.ui32HWPerfHostFilter,
	                         sApphints.eRGXActivePMConf);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "%s: PVRSRVRGXInitDevPart2KM failed (%d)",
				 __func__, eError));
		goto cleanup;
	}

#if defined(SUPPORT_VALIDATION)
	PVRSRVAppHintDumpState();
#endif

	eError = PVRSRV_OK;

cleanup:
	return eError;
}

/******************************************************************************
 End of file (rgxsrvinit.c)
******************************************************************************/
