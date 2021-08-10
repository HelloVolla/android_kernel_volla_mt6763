/*
 * Copyright (C) 2015 MediaTek Inc.
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

#ifndef MET_DRV_UDTL_H
#define MET_DRV_UDTL_H


/******************************************************************************
 * User-Defined Trace Line
 ******************************************************************************/
#define MET_UDTL_PROP_TYPE	ulong


/*
 * Declare PROPS Data Structure
 */



/*
 * Instance a PROPS Structure
 */
#define MET_UDTL_DEFINE_PROP(_PROPS_NAME_)

/*
 * PROPS Structure related
 */
#define MET_UDTL_PROP_DATATYPE(_PROPS_NAME_)
#define MET_UDTL_PROP_SIZE(_PROPS_NAME_) (0)


/*
 * Access PROPS Structure
 */
#define MET_UDTL_GET_PROP_POINTER(_PROPS_NAME_) (NULL)

#define MET_UDTL_SET_PROP(_PROPS_NAME_, _FIELD_NAME_, _VALUE_)

#define MET_UDTL_SET_PROP_POINTER(_PROPS_POINTER_NAME_, _FIELD_NAME_, _VALUE_)

#define MET_UDTL_CLEAR_PROP(_PROPS_NAME_)


/*
 * Functions Call
 */
#define MET_UDTL_TRACELINE_BEGIN(_PAGE_NAME_, _TRACELINE_NAME_, _FMTSTR_, ...)
#define MET_UDTL_TRACELINE_END(_PAGE_NAME_, _TRACELINE_NAME_, _FMTSTR_, ...)
#define MET_UDTL_TRACELINE_PROP(_PAGE_NAME_, _TRACELINE_NAME_, _PROPS_NAME_)
#define MET_UDTL_TRACELINE_DEBUG(_PAGE_NAME_, _TRACELINE_NAME_, _DEBUG_STR_)
#define MET_UDTL_IS_TRACELINE_ON(_PAGE_NAME_, _TRACELINE_NAME_)	(0)


#endif				/* MET_DRV_UDTL_H */
