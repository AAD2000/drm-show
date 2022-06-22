#include "../lib/lib.c"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

int main(int argc, char const *argv[])
{
    int fd = open("/dev/dri/renderD133",  O_RDWR);
    struct drm___create_bo info1 = {1024, 0xffffffff, DRM___BO_FLAGS_COHERENT | DRM___BO_FLAGS_CMA};
    int result = drm_gem_create_bo_ioctl(fd, &info1);
    printf("%d", result);

    return result;
}
