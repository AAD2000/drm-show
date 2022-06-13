#!/bin/bash

ssh aad@127.0.0.1 'bash -s' < clean.sh

scp my_ioctl.c aad@127.0.0.1:~/build/my_ioctl.c
scp my_xrt_mem.h aad@127.0.0.1:~/build/my_xrt_mem.h
scp zocl_error.h aad@127.0.0.1:~/build/zocl_error.h
scp xrt_error_code.h aad@127.0.0.1:~/build/xrt_error_code.h
scp mydriver.c aad@127.0.0.1:~/build/driver.c
scp mylib.h aad@127.0.0.1:~/build/mylib.h
scp Makefile aad@127.0.0.1:~/build/Makefile
scp mytest.c aad@127.0.0.1:~/build/mytest.c

ssh aad@127.0.0.1 'bash -s' < build.sh

