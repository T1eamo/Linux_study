# 指定目标系统
set(CMAKE_SYSTEM_NAME Linux)
# 指定目标平台
set(CMAKE_SYSTEM_PROCESSOR arm)
 
# 指定交叉编译工具链的根路径
set(CROSS_CHAIN_PATH /usr/local/arm/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu)
# 指定C编译器
set(CMAKE_C_COMPILER "${CROSS_CHAIN_PATH}/bin/aarch64-none-linux-gnu-gcc")
# 指定C++编译器
set(CMAKE_CXX_COMPILER "${CROSS_CHAIN_PATH}/bin/aarch64-none-linux-gnu-g++")
