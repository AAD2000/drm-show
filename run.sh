#!/bin/bash

ssh aad@127.0.0.1 'bash -s' < clean.sh

scp -r driver/my_ioctl.c aad@127.0.0.1:~/build/driver/my_ioctl.c
scp -r driver/my_xrt_mem.h aad@127.0.0.1:~/build/driver/my_xrt_mem.h
scp -r driver/__error.h aad@127.0.0.1:~/build/driver/__error.h
scp -r driver/xrt_error_code.h aad@127.0.0.1:~/build/driver/xrt_error_code.h
scp -r driver/mydriver.c aad@127.0.0.1:~/build/driver/driver.c
scp -r driver/Makefile aad@127.0.0.1:~/build/driver/Makefile
scp -r common/mylib.h aad@127.0.0.1:~/build/common/mylib.h
scp -r test/mytest.c aad@127.0.0.1:~/build/test/mytest.c
scp -r lib/lib.c aad@127.0.0.1:~/build/lib/lib.c

ssh aad@127.0.0.1 'bash -s' < build.sh

