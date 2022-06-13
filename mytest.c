#include "mylib.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

int main(int argc, char const *argv[])
{
    int fd = open("/dev/dri/card28",  O_RDWR);
    if (fd < 0) {
        return -10;
    }
    struct drm_zocl_create_bo info1 = {1024, 0xffffffff, DRM_ZOCL_BO_FLAGS_COHERENT | DRM_ZOCL_BO_FLAGS_CMA};
    int result = ioctl(fd, DRM_IOCTL_ZOCL_CREATE_BO, &info1);
    printf("%d", result);

    return 0;
}
