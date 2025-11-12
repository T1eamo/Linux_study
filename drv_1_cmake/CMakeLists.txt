cmake_minimum_required(VERSION 3.5)
project(HelloPro)

# 设置内核源码路径
set(KERNEL_DIR /home/yu/Documents/linux_sdk/kernel)

# 添加内核头文件路径
include_directories(${KERNEL_DIR}/include)
include_directories(${KERNEL_DIR}/arch/arm64/include)  # 添加架构相关的头文件路径
include_directories(${KERNEL_DIR}/arch/arm/include)
include_directories(/usr/include)

# 添加驱动源文件
add_executable(hello src/hello_drv.c)

# 链接内核模块
#target_link_libraries(hello -L${KERNEL_DIR} -lmodule)

# 添加测试程序
add_executable(hello_test src/hello_drv_test.c)
