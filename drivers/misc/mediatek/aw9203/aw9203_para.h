#ifndef __AW9203_PARA_H__
#define __AW9203_PARA_H__

/* aw9203 initial register @ mobile active */
#define AW9203_NORMAL_IDLECR                    0x1800	/* IDLE time setting */
#define AW9203_NORMAL_SLPR                      0x0023	/* touch key enable */
#define AW9203_NORMAL_SCFG1                     0x0084	/* scan time setting */
#define AW9203_NORMAL_SCFG2                     0x0287	/* bit0~3 is sense seting */
#define AW9203_NORMAL_OFR1                      0x1515	/* offset */
#define AW9203_NORMAL_OFR2                      0x1515	/* offset */
#define AW9203_NORMAL_OFR3                      0x1515	/* offset */
#define AW9203_NORMAL_THR0                      0x2328
#define AW9203_NORMAL_THR1                      0x2328
#define AW9203_NORMAL_THR2                      0x2328	/* S1 press thred setting */
#define AW9203_NORMAL_THR3                      0x2328	/* S2 press thred setting */
#define AW9203_NORMAL_THR4                      0x2328	/* S3 press thred setting */
#define AW9203_NORMAL_THR5                      0x2328
#define AW9203_NORMAL_SETCNT                    0x0202	/* debounce */
#define AW9203_NORMAL_BLCTH                     0x0402	/* base trace rate */
#define AW9203_NORMAL_AKSR                      0x001c	/* AKS */
#define AW9203_NORMAL_INTER                     0x001c	/* signel click interrupt */
#define AW9203_NORMAL_MPTR                      0x0005
#define AW9203_NORMAL_GDTR                      0x0000	/* gesture time setting */
#define AW9203_NORMAL_GDCFGR                    0x0000	/* gesture  key select */
#define AW9203_NORMAL_TAPR1                     0x0000	/* double click 1 */
#define AW9203_NORMAL_TAPR2                     0x0000	/* double click 2 */
#define AW9203_NORMAL_TDTR                      0x0000	/* double click time */
#define AW9203_NORMAL_GIER                      0x0000	/* gesture and double click enable */
#define AW9203_NORMAL_GCR                       0x0002	/* GCR */

/* AW9203 initial register @ mobile sleep */
#define AW9203_SLEEP_IDLECR                     0x1800	/* IDLE time setting */
#define AW9203_SLEEP_SLPR                       0x0023	/* touch key enable */
#define AW9203_SLEEP_SCFG1                      0x0084	/* scan time setting */
#define AW9203_SLEEP_SCFG2                      0x0287	/* bit0~3 is sense seting */
#define AW9203_SLEEP_OFR1                       0x0000	/* offset */
#define AW9203_SLEEP_OFR2                       0x0000	/* offset */
#define AW9203_SLEEP_OFR3                       0x0000	/* offset */
#define AW9203_SLEEP_THR0                       0x0a0c
#define AW9203_SLEEP_THR1                       0x0a0c
#define AW9203_SLEEP_THR2                       0x0a0c	/* S1 press thred setting */
#define AW9203_SLEEP_THR3                       0x0a0c	/* S2 press thred setting */
#define AW9203_SLEEP_THR4                       0x0a0c	/* S3 press thred setting */
#define AW9203_SLEEP_THR5                       0x0a0c
#define AW9203_SLEEP_SETCNT                     0x0202	/* debounce */
#define AW9203_SLEEP_BLCTH                      0x0402	/* base trace rate */
#define AW9203_SLEEP_AKSR                       0x001c	/* AKS */
#define AW9203_SLEEP_INTER                      0x0000	/* signel click interrupt */
#define AW9203_SLEEP_GDTR                       0x0000	/* gesture time setting */
#define AW9203_SLEEP_GDCFGR                     0x025f	/* gesture  key select */
#define AW9203_SLEEP_TAPR1                      0x0022	/* double click 1 */
#define AW9203_SLEEP_TAPR2                      0x0000	/* double click 2 */
#define AW9203_SLEEP_TDTR                       0x1932	/* double click time */
#define AW9203_SLEEP_GIER                       0x0010	/* gesture and double click enable */
#define AW9203_SLEEP_GCR                        0x0000	/* GCR */

#endif
