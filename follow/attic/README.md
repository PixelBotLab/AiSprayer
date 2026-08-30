P2 之前的设计骨架，已被 include/follow/ + src/ 里的实现取代：

- follow_odometry.hpp   滑窗里程计（"相对第一帧"），语义错误：地图持续吞实时帧会把
                        修正量自己拖向零。现在是冻结参考地图 + 逐帧配准，见 odometry.hpp。
- superpoint_rknn.hpp   NPU 前端草稿，接口已并入 frontend.hpp（CPU 默认，RKNN 在 HAS_RKNN 后）。
- main.cpp              整段注释掉的取流循环。真正的节点跟 Orbbec 取流层一起落地。

仓库里 follow/ 目前未纳入 git，所以这些文件没有直接删。确认不需要了就 rm -rf attic/。
