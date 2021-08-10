/*************************************************************************/ /*!
@Title          Rogue hw definitions (kernel mode)
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
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

#ifndef RGXDEFS_KM_H
#define RGXDEFS_KM_H

#if defined(RGX_BVNC_CORE_KM_HEADER) && defined(RGX_BNC_CONFIG_KM_HEADER)
#include RGX_BVNC_CORE_KM_HEADER
#include RGX_BNC_CONFIG_KM_HEADER
#endif

#define IMG_EXPLICIT_INCLUDE_HWDEFS
#if defined(__KERNEL__)
#include "rgx_cr_defs_km.h"
#endif
#undef IMG_EXPLICIT_INCLUDE_HWDEFS

#include "rgx_heap_firmware.h"

/* The following Macros are picked up through BVNC headers for PDUMP and
 * no hardware operations to be compatible with old build infrastructure.
 */
#if defined(PDUMP) || defined(NO_HARDWARE) || !defined(SUPPORT_MULTIBVNC_RUNTIME_BVNC_ACQUISITION)
/******************************************************************************
 * Check for valid B.X.N.C
 *****************************************************************************/
#if !defined(RGX_BVNC_KM_B) || !defined(RGX_BVNC_KM_V) || !defined(RGX_BVNC_KM_N) || !defined(RGX_BVNC_KM_C)
#error "Need to specify BVNC (RGX_BVNC_KM_B, RGX_BVNC_KM_V, RGX_BVNC_KM_N and RGX_BVNC_C)"
#endif
#endif

#if defined(PDUMP) || defined(NO_HARDWARE)
/* Check core/config compatibility */
#if (RGX_BVNC_KM_B != RGX_BNC_KM_B) || (RGX_BVNC_KM_N != RGX_BNC_KM_N) || (RGX_BVNC_KM_C != RGX_BNC_KM_C)
#error "BVNC headers are mismatching (KM core/config)"
#endif

#endif

/******************************************************************************
 * RGX Version name
 *****************************************************************************/
#define RGX_BVNC_ST2(S)	#S
#define RGX_BVNC_ST(S)		RGX_BVNC_ST2(S)
#define RGX_BVNC_KM			RGX_BVNC_ST(RGX_BVNC_KM_B) "." RGX_BVNC_ST(RGX_BVNC_KM_V) "." RGX_BVNC_ST(RGX_BVNC_KM_N) "." RGX_BVNC_ST(RGX_BVNC_KM_C)
#define RGX_BVNC_KM_V_ST	RGX_BVNC_ST(RGX_BVNC_KM_V)

/* Maximum string size is [bb.vvvp.nnnn.cccc\0], includes null char */
#define RGX_BVNC_STR_SIZE_MAX (2+1+4+1+4+1+4+1)
#define RGX_BVNC_STR_FMTSPEC  "%u.%u.%u.%u"
#define RGX_BVNC_STRP_FMTSPEC "%u.%up.%u.%u"


/******************************************************************************
 * RGX Defines
 *****************************************************************************/

#define BVNC_FIELD_MASK     ((1 << BVNC_FIELD_WIDTH) - 1)
#define C_POSITION          (0)
#define N_POSITION          ((C_POSITION) + (BVNC_FIELD_WIDTH))
#define V_POSITION          ((N_POSITION) + (BVNC_FIELD_WIDTH))
#define B_POSITION          ((V_POSITION) + (BVNC_FIELD_WIDTH))

#define B_POSTION_MASK      (((IMG_UINT64)(BVNC_FIELD_MASK) << (B_POSITION)))
#define V_POSTION_MASK      (((IMG_UINT64)(BVNC_FIELD_MASK) << (V_POSITION)))
#define N_POSTION_MASK      (((IMG_UINT64)(BVNC_FIELD_MASK) << (N_POSITION)))
#define C_POSTION_MASK      (((IMG_UINT64)(BVNC_FIELD_MASK) << (C_POSITION)))

#define GET_B(x)            (((x) & (B_POSTION_MASK)) >> (B_POSITION))
#define GET_V(x)            (((x) & (V_POSTION_MASK)) >> (V_POSITION))
#define GET_N(x)            (((x) & (N_POSTION_MASK)) >> (N_POSITION))
#define GET_C(x)            (((x) & (C_POSTION_MASK)) >> (C_POSITION))

#define BVNC_PACK(B,V,N,C)  ((((IMG_UINT64)B)) << (B_POSITION) | \
                             (((IMG_UINT64)V)) << (V_POSITION) | \
                             (((IMG_UINT64)N)) << (N_POSITION) | \
                             (((IMG_UINT64)C)) << (C_POSITION) \
                            )

#define RGX_CR_CORE_ID_CONFIG_N_SHIFT    (8U)
#define RGX_CR_CORE_ID_CONFIG_C_SHIFT    (0U)

#define RGX_CR_CORE_ID_CONFIG_N_CLRMSK   (0XFFFF00FFU)
#define RGX_CR_CORE_ID_CONFIG_C_CLRMSK   (0XFFFFFF00U)

/* The default number of OSID is 1, higher number implies VZ enabled firmware */
#if !defined(RGXFW_NATIVE) && defined(PVRSRV_VZ_NUM_OSID) && (PVRSRV_VZ_NUM_OSID + 1U > 1U)
#define RGXFW_NUM_OS PVRSRV_VZ_NUM_OSID
#else
#define RGXFW_NUM_OS 1U
#endif

#define RGXFW_MAX_NUM_OS                                  (8U)
#define RGXFW_HYPERVISOR_OS                               (0U)
#define RGXFW_GUEST_OSID_START                            (1U)

#define RGXFW_THREAD_0                                    (0U)
#define RGXFW_THREAD_1                                    (1U)

/* META cores (required for the RGX_FEATURE_META) */
#define MTP218   (1)
#define MTP219   (2)
#define LTP218   (3)
#define LTP217   (4)

/* META Core memory feature depending on META variants */
#define RGX_META_COREMEM_32K      (32*1024)
#define RGX_META_COREMEM_48K      (48*1024)
#define RGX_META_COREMEM_64K      (64*1024)
#define RGX_META_COREMEM_96K      (96*1024)
#define RGX_META_COREMEM_128K     (128*1024)
#define RGX_META_COREMEM_256K     (256*1024)

#if !defined(SUPPORT_MULTIBVNC)
#if (!defined(SUPPORT_TRUSTED_DEVICE) || defined(RGX_FEATURE_META_DMA)) && \
    (defined(RGX_FEATURE_META_COREMEM_SIZE) && RGX_FEATURE_META_COREMEM_SIZE != 0)
#define RGX_META_COREMEM_SIZE     (RGX_FEATURE_META_COREMEM_SIZE*1024U)
#define RGX_META_COREMEM          (1)
#define RGX_META_COREMEM_CODE     (1)
#if !defined(FIX_HW_BRN_50767) && (RGXFW_NUM_OS == 1)
#define RGX_META_COREMEM_DATA     (1)
#endif
#else
#undef SUPPORT_META_COREMEM
#undef RGX_FEATURE_META_COREMEM_SIZE
#undef RGX_FEATURE_META_DMA
#define RGX_FEATURE_META_COREMEM_SIZE (0)
#define RGX_META_COREMEM_SIZE         (0)
#endif
#endif

#define GET_ROGUE_CACHE_LINE_SIZE(x)    ((((IMG_INT32)x) > 0) ? ((x)/8) : (0))


#define MAX_HW_TA3DCONTEXTS	2U

#define RGX_CR_SOFT_RESET_DUST_n_CORE_EN	(RGX_CR_SOFT_RESET_DUST_A_CORE_EN | \
											 RGX_CR_SOFT_RESET_DUST_B_CORE_EN | \
											 RGX_CR_SOFT_RESET_DUST_C_CORE_EN | \
											 RGX_CR_SOFT_RESET_DUST_D_CORE_EN | \
											 RGX_CR_SOFT_RESET_DUST_E_CORE_EN | \
											 RGX_CR_SOFT_RESET_DUST_F_CORE_EN | \
											 RGX_CR_SOFT_RESET_DUST_G_CORE_EN | \
											 RGX_CR_SOFT_RESET_DUST_H_CORE_EN)

/* SOFT_RESET Rascal and DUSTs bits */
#define RGX_CR_SOFT_RESET_RASCALDUSTS_EN	(RGX_CR_SOFT_RESET_RASCAL_CORE_EN | \
											 RGX_CR_SOFT_RESET_DUST_n_CORE_EN)




/* SOFT_RESET steps as defined in the TRM */
#define RGX_S7_SOFT_RESET_DUSTS (RGX_CR_SOFT_RESET_DUST_n_CORE_EN)

#define RGX_S7_SOFT_RESET_JONES (RGX_CR_SOFT_RESET_PM_EN  | \
                                 RGX_CR_SOFT_RESET_VDM_EN | \
                                 RGX_CR_SOFT_RESET_ISP_EN)

#define RGX_S7_SOFT_RESET_JONES_ALL (RGX_S7_SOFT_RESET_JONES  | \
                                     RGX_CR_SOFT_RESET_BIF_EN | \
                                     RGX_CR_SOFT_RESET_SLC_EN | \
                                     RGX_CR_SOFT_RESET_GARTEN_EN)

#define RGX_S7_SOFT_RESET2 (RGX_CR_SOFT_RESET2_BLACKPEARL_EN | \
                            RGX_CR_SOFT_RESET2_PIXEL_EN | \
                            RGX_CR_SOFT_RESET2_CDM_EN | \
                            RGX_CR_SOFT_RESET2_VERTEX_EN)



#define RGX_BIF_PM_PHYSICAL_PAGE_ALIGNSHIFT		(12)
#define RGX_BIF_PM_PHYSICAL_PAGE_SIZE			(1 << RGX_BIF_PM_PHYSICAL_PAGE_ALIGNSHIFT)

#define RGX_BIF_PM_VIRTUAL_PAGE_ALIGNSHIFT		(14)
#define RGX_BIF_PM_VIRTUAL_PAGE_SIZE			(1 << RGX_BIF_PM_VIRTUAL_PAGE_ALIGNSHIFT)

#define RGX_BIF_PM_FREELIST_BASE_ADDR_ALIGNSIZE	(16U)

/* To get the number of required Dusts, divide the number of
 * clusters by 2 and round up
 */
#define RGX_REQ_NUM_DUSTS(CLUSTERS)    ((CLUSTERS + 1U) / 2U)

/* To get the number of required Bernado/Phantom(s), divide
 * the number of clusters by 4 and round up
 */
#define RGX_REQ_NUM_PHANTOMS(CLUSTERS) ((CLUSTERS + 3U) / 4U)
#define RGX_REQ_NUM_BERNADOS(CLUSTERS) ((CLUSTERS + 3U) / 4U)
#define RGX_REQ_NUM_BLACKPEARLS(CLUSTERS) ((CLUSTERS + 3U) / 4U)

#if !defined(SUPPORT_MULTIBVNC)
# define RGX_NUM_PHANTOMS (RGX_REQ_NUM_PHANTOMS(RGX_FEATURE_NUM_CLUSTERS))
#endif


/* RGX_FEATURE_CDM_CONTROL_STREAM_FORMAT is not defined for format 1 cores (so define it now). */
#if !defined(RGX_FEATURE_CDM_CONTROL_STREAM_FORMAT)
#define RGX_FEATURE_CDM_CONTROL_STREAM_FORMAT (1)
#endif

/* META second thread feature depending on META variants and
 * available CoreMem
 */
#if defined(RGX_FEATURE_META) && (RGX_FEATURE_META == MTP218 || RGX_FEATURE_META == MTP219) && defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE) && (RGX_FEATURE_META_COREMEM_SIZE == 256)
#define RGXFW_META_SUPPORT_2ND_THREAD
#endif


/******************************************************************************
 * WA HWBRNs
 *****************************************************************************/
#if defined(FIX_HW_BRN_36492)

#undef RGX_CR_SOFT_RESET_SLC_EN
#undef RGX_CR_SOFT_RESET_SLC_CLRMSK
#undef RGX_CR_SOFT_RESET_SLC_SHIFT

/* Remove the SOFT_RESET_SLC_EN bit from SOFT_RESET_MASKFULL */
#undef RGX_CR_SOFT_RESET_MASKFULL
#define RGX_CR_SOFT_RESET_MASKFULL IMG_UINT64_C(0x000001FFF7FFFC1D)

#endif /* FIX_HW_BRN_36492 */


#if defined(RGX_CR_JONES_IDLE_MASKFULL)
/* Workaround for HW BRN 57289 */
#if (RGX_CR_JONES_IDLE_MASKFULL != 0x0000000000007FFF)
#error This WA must be updated if RGX_CR_JONES_IDLE is expanded!!!
#endif
#undef RGX_CR_JONES_IDLE_MASKFULL
#undef RGX_CR_JONES_IDLE_TDM_SHIFT
#undef RGX_CR_JONES_IDLE_TDM_CLRMSK
#undef RGX_CR_JONES_IDLE_TDM_EN
#define RGX_CR_JONES_IDLE_MASKFULL                        (IMG_UINT64_C(0x0000000000003FFF))
#endif

#if !defined(SUPPORT_MULTIBVNC)
#if !defined(RGX_FEATURE_SLC_SIZE_IN_BYTES)
#if defined(RGX_FEATURE_SLC_SIZE_IN_KILOBYTES)
#define RGX_FEATURE_SLC_SIZE_IN_BYTES (RGX_FEATURE_SLC_SIZE_IN_KILOBYTES * 1024)
#else
#define RGX_FEATURE_SLC_SIZE_IN_BYTES (0)
#endif
#endif
#endif

#if !defined(SUPPORT_MULTIBVNC)

#if defined(RGX_FEATURE_ROGUEXE)
#define RGX_NUM_RASTERISATION_MODULES	RGX_FEATURE_NUM_CLUSTERS
#else
#define RGX_NUM_RASTERISATION_MODULES	RGX_NUM_PHANTOMS
#endif

#endif /* defined(SUPPORT_MULTIBVNC) */

/* GPU CR timer tick in GPU cycles */
#define RGX_CRTIME_TICK_IN_CYCLES (256U)

#endif /* RGXDEFS_KM_H */
