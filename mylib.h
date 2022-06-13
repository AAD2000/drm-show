#ifndef __KERNEL__
#include <stdint.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#else
#include <uapi/drm/drm_mode.h>
#endif /* !__KERNEL__ */

#define DRM_ZOCL_BO_FLAGS_HOST_BO    (0x1 << 26)
#define DRM_ZOCL_BO_FLAGS_COHERENT   (0x1 << 27)
#define DRM_ZOCL_BO_FLAGS_CMA        (0x1 << 28)
#define DRM_ZOCL_BO_FLAGS_SVM        (0x1 << 29)
#define DRM_ZOCL_BO_FLAGS_USERPTR    (0x1 << 30)
#define DRM_ZOCL_BO_FLAGS_EXECBUF    (0x1 << 31)

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
                                       struct drm_zocl_create_bo)