#ifndef __KERNEL__
#include <stdint.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#else
#include <uapi/drm/drm_mode.h>
#endif /* !__KERNEL__ */

#define DRM___BO_FLAGS_HOST_BO    (0x1 << 26)
#define DRM___BO_FLAGS_COHERENT   (0x1 << 27)
#define DRM___BO_FLAGS_CMA        (0x1 << 28)
#define DRM___BO_FLAGS_SVM        (0x1 << 29)
#define DRM___BO_FLAGS_USERPTR    (0x1 << 30)
#define DRM___BO_FLAGS_EXECBUF    (0x1 << 31)

/*
 * enum drm___ops - ioctl command code enumerations
 */
enum drm___ops {
	/* Buffer creation */
	DRM___CREATE_BO = 0
};

/**
 * struct drm___create_bo - Create buffer object
 * used with DRM_IOCTL___CREATE_BO ioctl
 *
 * @size:       Requested size of the buffer object
 * @handle:     bo handle returned by the driver
 * @flags:      DRM___BO_XXX flags
 */
struct drm___create_bo {
	uint64_t size;
	uint32_t handle;
	uint32_t flags;
};



#define DRM_IOCTL___CREATE_BO       DRM_IOWR(DRM_COMMAND_BASE + \
                                       DRM___CREATE_BO,     \
                                       struct drm___create_bo)