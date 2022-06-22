#include "../common/mylib.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

/**
 * Drm gem create bo wrapper
 */
int drm_gem_create_bo_ioctl(int fd, struct drm___create_bo* object) {
  int result = ioctl(fd, DRM_IOCTL___CREATE_BO, &object);
  if (result != 0)
    printf("Error while calling ioctl: %d\n", result);
  else
    printf("Success\n");

  return result;
}

