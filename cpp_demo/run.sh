# 1. 删除旧的 build 目录
rm -rf build

# 2. 重新配置并编译
cmake -B build -S .
cmake --build build

# 3. 运行程序
./build/cpp_demo
