/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*=============================================================================
 *							  Include Files
 *===========================================================================
 */
#include "val_types_private.h"
#include "val_api_private.h"
/* #include "mfv_reg.h" */
/* #include "mfv_config.h" */
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/semaphore.h>

#define MAX_HEAP_SIZE (0x1000000)
#ifdef _6573_FPGA
#define GMC 0xC8000000
#else /* _6573_REAL_CHIP E1 address */
#define GMC 0x40000000
#endif

/*=============================================================================
 *							  Function Body
 *===========================================================================
 */

enum VAL_RESULT_T eVideoMemAlloc(struct VAL_MEMORY_T *a_prPaam,
				 unsigned int a_u4ParamSize)
{

	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoMemFree(struct VAL_MEMORY_T *a_prParam,
				unsigned int a_u4ParamSize)
{
	/* pr_debug("!!Free Mem Size:%d!!\n",a_prParam->u4MemSize); */

	return VAL_RESULT_NO_ERROR;
}

/* mimic internal memory buffer, 128K bytes. */
enum VAL_RESULT_T eVideoIntMemUsed(struct VAL_INTMEM_T *a_prParam,
				   unsigned int a_u4ParamSize)
{

	return VAL_RESULT_NO_ERROR;
}

/* mimic internal memory buffer, 128K bytes. */
enum VAL_RESULT_T eVideoIntMemAlloc(struct VAL_INTMEM_T *a_prParam,
				    unsigned int a_u4ParamSize)
{

	return VAL_RESULT_NO_ERROR;
}

/* mimic internal memory buffer, 128K bytes. */
enum VAL_RESULT_T eVideoIntMemFree(struct VAL_INTMEM_T *a_prParam,
				   unsigned int a_u4ParamSize)
{

	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoCreateEvent(struct VAL_EVENT_T *a_prParam,
				    unsigned int a_u4ParamSize)
{
	wait_queue_head_t *pWaitQueue;
	unsigned char *pFlag;

	pWaitQueue = kmalloc(sizeof(wait_queue_head_t), GFP_ATOMIC);
	pFlag = kmalloc(sizeof(unsigned char), GFP_ATOMIC);
	if (pWaitQueue != VAL_NULL) {
		init_waitqueue_head(pWaitQueue);
		a_prParam->pvWaitQueue = (void *)pWaitQueue;
	} else {
		pr_info("[VCODEC] Event wait Queue failed to create\n");
	}
	if (pFlag != VAL_NULL) {
		a_prParam->pvReserved = (void *)pFlag;
		*((unsigned char *)a_prParam->pvReserved) = VAL_FALSE;
	} else {
		pr_info("[VCODEC] Event flag failed to create\n");
	}

	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoCloseEvent(struct VAL_EVENT_T *a_prParam,
				   unsigned int a_u4ParamSize)
{
	wait_queue_head_t *pWaitQueue;
	unsigned char *pFlag;

	pWaitQueue = (wait_queue_head_t *)a_prParam->pvWaitQueue;
	pFlag = (unsigned char *)a_prParam->pvReserved;

	kfree(pWaitQueue);
	kfree(pFlag);

	a_prParam->pvWaitQueue = VAL_NULL;
	a_prParam->pvReserved = VAL_NULL;
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoWaitEvent(struct VAL_EVENT_T *a_prParam,
				  unsigned int a_u4ParamSize)
{
	wait_queue_head_t *pWaitQueue;
	long timeout_jiff, i4Ret;
	enum VAL_RESULT_T status;

	pWaitQueue = (wait_queue_head_t *)a_prParam->pvWaitQueue;
	timeout_jiff = (a_prParam->u4TimeoutMs) * HZ / 1000;
	/* pr_debug("[MFV]eVideoWaitEvent,a_prParam->u4TimeoutMs=%d,
	 * timeout = %ld\n",
	 * a_prParam->u4TimeoutMs,timeout_jiff);
	 */
	timeout_jiff = (timeout_jiff == 0) ? 1 : timeout_jiff;
	i4Ret = wait_event_interruptible_timeout(
	    *pWaitQueue,
	    *((unsigned char *)
		  a_prParam->pvReserved) /*g_mflexvideo_interrupt_handler */,
	    timeout_jiff);
	if (i4Ret == 0) {
		pr_info("[VCODEC] %s timeout: %d ms", __func__,
			a_prParam->u4TimeoutMs);
		status = VAL_RESULT_INVALID_ISR; /* timeout */
	} else if (-ERESTARTSYS == i4Ret) {
		pr_info("[VCODEC] %s wake up by ERESTARTSYS", __func__);
		status = VAL_RESULT_RESTARTSYS;
	} else if (i4Ret > 0) {
		status = VAL_RESULT_NO_ERROR;
	} else {
		pr_info("[VCODEC] %s wake up by %ld", __func__, i4Ret);
		status = VAL_RESULT_NO_ERROR;
	}
	*((unsigned char *)a_prParam->pvReserved) = VAL_FALSE;
	return status;
}

enum VAL_RESULT_T eVideoSetEvent(struct VAL_EVENT_T *a_prParam,
				 unsigned int a_u4ParamSize)
{
	wait_queue_head_t *pWaitQueue;
	/* pr_debug("[MFV]eVideoSetEvent\n"); */
	pWaitQueue = (wait_queue_head_t *)a_prParam->pvWaitQueue;
	if (a_prParam->pvReserved != VAL_NULL) {
		/* Add one line comment for avoid kernel coding style,
		 * WARNING:BRACES:
		 */
		*((unsigned char *)a_prParam->pvReserved) = VAL_TRUE;
	} else {
		pr_info("[VCODEC] Event flag should not be null\n");
	}
	if (pWaitQueue != VAL_NULL) {
		/* Add one line comment for avoid kernel coding style,
		 * WARNING:BRACES:
		 */
		wake_up_interruptible(pWaitQueue);
	} else {
		pr_info("[VCODEC] Wait Queue should not be null\n");
	}
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoCreateMutex(struct VAL_MUTEX_T *a_prParam,
				    unsigned int a_u4ParamSize)
{
	struct semaphore *pLock;

	pLock = kmalloc(sizeof(struct semaphore), GFP_ATOMIC);
	if (pLock != VAL_NULL) {
		/* Add one line comment for avoid kernel coding style,
		 * WARNING:BRACES:
		 */
		a_prParam->pvMutex = (void *)pLock;
	} else {
		pr_info("[VCODEC] Unable to create mutex!\n");
	}
	/* init_MUTEX(pLock); */
	sema_init(pLock, 1);
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoCloseMutex(struct VAL_MUTEX_T *a_prParam,
				   unsigned int a_u4ParamSize)
{
	if (a_prParam->pvMutex != VAL_NULL) {
		/* Add one line comment for avoid kernel coding style,
		 * WARNING:BRACES:
		 */
		kfree(a_prParam->pvMutex);
	}
	a_prParam->pvMutex = VAL_NULL;
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoWaitMutex(struct VAL_MUTEX_T *a_prParam,
				  unsigned int a_u4ParamSize)
{
	struct semaphore *pLock;

	pLock = (struct semaphore *)a_prParam->pvMutex;
	down(pLock);
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoReleaseMutex(struct VAL_MUTEX_T *a_prParam,
				     unsigned int a_u4ParamSize)
{
	struct semaphore *pLock;

	pLock = (struct semaphore *)a_prParam->pvMutex;
	up(pLock);
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoMemSet(struct VAL_MEMORY_T *a_prParam,
			       unsigned int a_u4ParamSize, int a_u4Value,
			       unsigned int a_u4Size)
{
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoMemCpy(struct VAL_MEMORY_T *a_prParamDst,
			       unsigned int a_u4ParamDstSize,
			       struct VAL_MEMORY_T *a_prParamSrc,
			       unsigned int a_u4ParamSrcSize,
			       unsigned int a_u4Size)
{
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoMemCmp(struct VAL_MEMORY_T *a_prParamSrc1,
			       unsigned int a_u4ParamSrc1Size,
			       struct VAL_MEMORY_T *a_prParamSrc2,
			       unsigned int a_u4ParamSrc2Size,
			       unsigned int a_u4Size)
{
	return VAL_RESULT_NO_ERROR;
}

enum VAL_RESULT_T eVideoGetTimeOfDay(struct VAL_TIME_T *a_prParam,
				     unsigned int a_u4ParamSize)
{
	struct timeval t1;

	do_gettimeofday(&t1);
	a_prParam->u4Sec = t1.tv_sec;
	a_prParam->u4uSec = t1.tv_usec;
	return VAL_RESULT_NO_ERROR;
}
