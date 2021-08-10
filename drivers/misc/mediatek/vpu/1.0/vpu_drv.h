/*
 * Copyright (C) 2016 MediaTek Inc.
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

#ifndef __VPU_DRV_H__
#define __VPU_DRV_H__

#include <linux/types.h>

#define VPU_MAX_NUM_PORTS 32
#define VPU_MAX_NUM_PROPS 32
typedef uint8_t vpu_id_t;

/* the last byte of string must be '/0' */
/*typedef char vpu_name_t[32];*/
#define VPU_NAME_SIZE 32

/**
 * Documentation index:
 *   S1. Introduction
 *   S2. Requirement
 *   S3. Sample Use Cases
 */

/**
 * S1. Introduction
 * VPU driver is a transparent platform for data exchange with VPU firmware.
 * VPU firmware can dynamically load an algorithm and do image post-processing.
 *
 * VPU driver implements a model based on aspect of algorithm's requirements.
 * An algorithm needs the buffers of input and output, and execution arguments.
 * For all mentionedabove, VPU driver defines 'Port' to describe the buffers
 * of input and output, and 'Info' to describe the specification of algorithm.
 * According the 'Port' and 'Info', a user could enque requests for doing
 * image post-processing. The diagram is as follows:
 *
 *                 +---------------+
 *                 |     algo      |
 *                 |    ------     |
 *   input port1-> | [info1]       | ->output port1
 *                 | [info2]       |
 *   input port2-> | [info...]     |
 *                 +---------------+
 *
 * With Algo's properties, a user can get enough information to do processing,
 * and assign the buffers to the matching ports.
 * Moreover, a user algo can specify execution arguments to a request.
 *
 *   +------------------------+
 *   |        request         |
 *   |     -------------      |
 *   | [buffer1]=>input port1 |
 *   | [buffer2]=>input port2 |
 *   | [buffer3]=>input port3 |
 *   | [setting1]             |
 *   | [setting2]             |
 *   | [setting...]           |
 *   +------------------------+
 *
 */

/**
 * S2. Requirement
 * 1. The processing order is FIFO. User should deque the request in order.
 * 2. The buffer address must be accessible by VPU. Use iommu to remap address
 * to the specific region.
 *
 */

/**
 * S3. Sample Use Cases
 * Provide 4 essential ioctl commands:
 * - VPU_IOCTL_GET_ALGO_INFO: get algo's port and info.
 *
 *     struct vpu_algo algo;
 *     strncpy(algo_n->name, "algo_name", sizeof(algo_n->name));
 *     ioctl(fd, VPU_IOCTL_GET_ALGO_INFO, algo);
 *
 * - VPU_IOCTL_ENQUE_REQUEST: enque a request to user’s own queue.
 *
 *     struct vpu_request req;
 *     struct vpu_buffer *buf;
 *     req->algo_id = algo->id;
 *     req->buffer_count = 1;
 *     buf = &req->buffers[0];
 *     buf->format = VPU_BUF_FORMAT_IMG_Y8;
 *     buf->width = 640;
 *     buf->height = 360;
 *     buf->plane_count = 1;
 *     ioctl(fd, VPU_IOCTL_ENQ_REQUEST, req);
 *
 * - VPU_IOCTL_DEQUE_REQUEST: wait for request done, and get processing result.
 *
 *     struct vpu_request req;
 *     ioctl(fd, VPU_IOCTL_DEQUE_REQUEST, req);
 *
 * - VPU_IOCTL_FLUSH_REQUEST: flush all running request, and return failure if
 * not finished
 *
 *     ioctl(fd, VPU_IOCTL_FLUSH_REQUEST, 0);
 *
 * - VPU_IOCTL_SET_POWER: request power mode and the performance.
 *
 *     struct vpu_power power;
 *     power.mode = VPU_POWER_MODE_DYNAMIC;
 *     power.opp = VPU_POWER_OPP_UNREQUEST;
 *     ioctl(fd, VPU_IOCTL_SET_POWER, power);
 *
 */

/*---------------------------------------------------------------------------*/
/*  VPU Property                                                             */
/*---------------------------------------------------------------------------*/
enum vpu_prop_type {
	VPU_PROP_TYPE_CHAR,
	VPU_PROP_TYPE_INT32,
	VPU_PROP_TYPE_INT64,
	VPU_PROP_TYPE_FLOAT,
	VPU_PROP_TYPE_DOUBLE,
	VPU_NUM_PROP_TYPES
};

enum vpu_prop_access {
	VPU_PROP_ACCESS_RDONLY,
	VPU_PROP_ACCESS_RDWR
};

/*
 * The description of properties contains the information about property values,
 * which are stored as compact memory. With the offset, it can get the specific
 * value
 * from compact data.
 *
 * The example of struct vpu_prop_desc is as follows:
 *   +--------+---------------------+--------+--------+-------+--------+
 *   |   id   | name                | offset | type   | count | access |
 *   +--------+---------------------+--------+--------+-------+--------+
 *   |   0    | algo_version        | 0      | int32  | 1     | RDONLY |
 *   +--------+---------------------+--------+--------+-------+--------+
 *   |   1    | field_1             | 4      | int32  | 2     | RDWR   |
 *   +--------+---------------------+--------+--------+-------+--------+
 *   |   2    | field_2             | 12     | int64  | 1     | RDWR   |
 *   +--------+---------------------+--------+--------+-------+--------+
 *
 * Use a buffer to store all property data, which is a compact-format data.
 * The buffer's layout is described by prop_desc, using the offset could get the
 * specific data.
 *
 * The example of compact-format memory is as follows:
 *   +--------+--------+--------+--------+--------+
 *   |  0~3   |  4~7   |  8~11  |  12~15 |  16~23 |
 *   +--------+--------+--------+--------+--------+
 *   |alg_vers|    field_1      |    field_2      |
 *   +--------+--------+--------+--------+--------+
 *
 */
struct vpu_prop_desc {
	vpu_id_t id;
	uint8_t type;    /* the property's data type */
	uint8_t access;  /* directional data exchange */
	uint32_t offset; /* offset = previous offset + previous size */
	uint32_t count;  /* size = sizeof(type) x count */
	char name[VPU_NAME_SIZE];
};

/*---------------------------------------------------------------------------*/
/*  VPU Ports                                                                */
/*---------------------------------------------------------------------------*/
enum vpu_port_usage {
	VPU_PORT_USAGE_IMAGE,
	VPU_PORT_USAGE_DATA,
	VPU_NUM_PORT_USAGES
};

enum vpu_port_dir {
	VPU_PORT_DIR_IN,
	VPU_PORT_DIR_OUT,
	VPU_PORT_DIR_IN_OUT,
	VPU_NUM_PORT_DIRS
};

/*
 * The ports contains the information about algorithm's input and output.
 * The each buffer on the vpu_request should be assigned a port id,
 * to let algorithm recognize every buffer's purpose.
 *
 * The example of vpu_port table is as follows:
 *   +--------+---------------------+--------+--------+
 *   |   id   | name                | type   | dir    |
 *   +--------+---------------------+--------+--------+
 *   |   0    | image-in            | IMAGE  | IN     |
 *   +--------+---------------------+--------+--------+
 *   |   1    | data-in             | DATA   | IN     |
 *   +--------+---------------------+--------+--------+
 *   |   2    | image-out           | IMAGE  | OUT    |
 *   +--------+---------------------+--------+--------+
 *   |   3    | image-temp          | IMAGE  | INOUT  |
 *   +--------+---------------------+--------+--------+
 *
 */
struct vpu_port {
	vpu_id_t id;
	uint8_t usage;
	uint8_t dir;
	char name[VPU_NAME_SIZE];
};

/*---------------------------------------------------------------------------*/
/*  VPU Algo                                                                 */
/*---------------------------------------------------------------------------*/
struct vpu_algo {
	vpu_id_t id;
	uint8_t port_count;
	uint8_t info_desc_count;
	uint8_t sett_desc_count;
	uint32_t info_length; /* the size of info data buffer */
	uint32_t sett_length;
	uint32_t bin_length;
	uint64_t info_ptr; /* the pointer to info data buffer */
	uint64_t bin_ptr;  /* mva of algo bin, which is accessible by VPU */
	char name[VPU_NAME_SIZE];
	struct vpu_prop_desc info_descs[VPU_MAX_NUM_PROPS];
	struct vpu_prop_desc sett_descs[VPU_MAX_NUM_PROPS];
	struct vpu_port ports[VPU_MAX_NUM_PORTS];
};

/*---------------------------------------------------------------------------*/
/*  VPU Register                                                             */
/*---------------------------------------------------------------------------*/
struct vpu_reg_value {
	uint32_t field;
	uint32_t value;
};

struct vpu_reg_values {
	uint8_t reg_count;
	struct vpu_reg_value *regs;
};

/*---------------------------------------------------------------------------*/
/*  VPU Power                                                                */
/*---------------------------------------------------------------------------*/

/*
 * Provide two power modes:
 * - dynamic: power-saving mode, it's on request to power on device.
 * - on: power on immediately
 */
enum vpu_power_mode {
	VPU_POWER_MODE_DYNAMIC,
	VPU_POWER_MODE_ON,
};

/*
 * Provide a set of OPPs(operation performance point)
 * The default opp is at the minimun performance, and users could request the
 * performance.
 */
enum vpu_power_opp {
	VPU_POWER_OPP_UNREQUEST = 0xFF,
};

struct vpu_power {
	uint8_t mode;
	uint8_t opp;
};

/*---------------------------------------------------------------------------*/
/*  VPU Plane                                                                */
/*---------------------------------------------------------------------------*/
struct vpu_plane {
	uint32_t stride; /* if buffer type is image */
	uint32_t length;
	uint64_t ptr; /* mva which is accessible by VPU */
};

enum vpu_buf_format {
	VPU_BUF_FORMAT_DATA,
	VPU_BUF_FORMAT_IMG_Y8,
	VPU_BUF_FORMAT_IMG_YV12,
	VPU_BUF_FORMAT_IMG_NV21,
	VPU_BUF_FORMAT_IMG_YUY2,
	VPU_BUF_FORMAT_IMPL_DEFINED = 0xFF,
};

struct vpu_buffer {
	vpu_id_t port_id;
	uint8_t format;
	uint8_t plane_count;
	uint32_t width;
	uint32_t height;
	struct vpu_plane planes[3];
};

enum vpu_req_status {
	VPU_REQ_STATUS_SUCCESS,
	VPU_REQ_STATUS_BUSY,
	VPU_REQ_STATUS_TIMEOUT,
	VPU_REQ_STATUS_INVALID,
	VPU_REQ_STATUS_FLUSH,
	VPU_REQ_STATUS_FAILURE,
};

struct vpu_request {
	vpu_id_t algo_id;
	uint8_t status;
	uint8_t buffer_count;
	uint32_t sett_length;
	uint64_t sett_ptr; /* pointer to the request setting */
	uint64_t priv;     /* reserved for user */
	struct vpu_buffer buffers[VPU_MAX_NUM_PORTS];
};

/*---------------------------------------------------------------------------*/
/*  IOCTL Command                                                            */
/*---------------------------------------------------------------------------*/
#define VPU_MAGICNO                 'v'
#define VPU_IOCTL_SET_POWER         _IOW(VPU_MAGICNO,   0, int)
#define VPU_IOCTL_ENQUE_REQUEST     _IOW(VPU_MAGICNO,   1, int)
#define VPU_IOCTL_DEQUE_REQUEST     _IOWR(VPU_MAGICNO,  2, int)
#define VPU_IOCTL_FLUSH_REQUEST     _IOW(VPU_MAGICNO,   3, int)
#define VPU_IOCTL_GET_ALGO_INFO     _IOWR(VPU_MAGICNO,  4, int)
#define VPU_IOCTL_LOCK              _IOW(VPU_MAGICNO,   5, int)
#define VPU_IOCTL_UNLOCK            _IOW(VPU_MAGICNO,   6, int)
#define VPU_IOCTL_LOAD_ALG          _IOW(VPU_MAGICNO,   7, int)
#define VPU_IOCTL_REG_WRITE         _IOW(VPU_MAGICNO,   8, int)
#define VPU_IOCTL_REG_READ          _IOWR(VPU_MAGICNO,  9, int)

#endif
