#!/bin/bash

# 1. 定义安装路径为当前目录下的 install 文件夹
# $(pwd) 会获取当前脚本执行的绝对路径
INSTALL_PATH="$(pwd)/install"

echo "🚀 开始编译项目..."
echo "📍 安装目标路径: $INSTALL_PATH"

# 2. 创建并进入 build 目录
mkdir -p build
cd build

# 3. 运行 CMake
# -DCMAKE_INSTALL_PREFIX 指定了安装位置
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" ..

# 4. 编译 (使用所有 CPU 核心)
make -j$(nproc)

# 5. 执行安装
# 这步会将生成的库和头文件拷贝到指定的 install 目录
make install

echo "✅ 任务完成！请查看 $INSTALL_PATH 目录。"