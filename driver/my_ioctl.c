#ifndef __ZYNQ_IOCTL_H__
#define __ZYNQ_IOCTL_H__

#ifndef __KERNEL__
#include <stdint.h>
#include <libdrm/drm.h>
#include <libdrm/drm_mode.h>
#include <drm/drm_ioctl.h>
#else
#include <uapi/drm/drm_mode.h>
#endif /* !__KERNEL__ */

/*
 * enum drm_zocl_ops - ioctl command code enumerations
 */
enum drm_zocl_ops {
	/* Buffer creation */
	DRM_ZOCL_CREATE_BO = 0
};
/**
 * struct drm_zocl_create_bo - Create buffer object
 * used with DRM_IOCTL_ZOCL_CREATE_BO ioctl
 *
 * @size:       Requested size of the buffer object
 * @handle:     bo handle returned by the driver
 * @flags:      DRM_ZOCL_BO_XXX flags
 */
struct drm_zocl_create_bo {
	uint64_t size;
	uint32_t handle;
	uint32_t flags;
};

#define DRM_IOCTL_ZOCL_CREATE_BO       DRM_IOWR(DRM_COMMAND_BASE + \
                                       DRM_ZOCL_CREATE_BO,     \
                                       struct drm_zo cl_create_bo)