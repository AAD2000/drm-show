#include "mylib.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

int main(int argc, char const *argv[])
{
    int fd = open("/dev/dri/card0",  O_RDWR);
    if (fd < 0) {
        return -10;
    }
    struct drm_zocl_create_bo info1 = {1024, 0xffffffff, 0};
    int result = ioctl(fd, DRM_IOCTL_ZOCL_CREATE_BO, &info1);
    printf("%d", result);

    return 0;
}
