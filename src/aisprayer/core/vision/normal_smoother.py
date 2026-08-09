import numpy as np

class PathNormalSmoother:
    """
    轨迹法向量平滑器 (独立模块)
    功能：对生成的 3D 喷涂轨迹中的法向量进行滤波平滑。
    目的：消除由于 3D 网格表面高频噪声或微小凹凸导致的法向量剧烈抖动，
          防止机器人在执行喷涂动作时手腕关节产生高频抽搐。
    """
    
    def __init__(self, window_size=5):
        """
        初始化平滑器。
        :param window_size: 移动平均的窗口大小。窗口越大，法向变化越平缓，但也可能丢失局部特征。
                            建议设置为奇数（如 3, 5, 7）。
        """
        self.window_size = window_size

    def smooth(self, paths: list) -> list:
        """
        对传入的路径集合执行 1D 法向量平滑。
        
        算法说明：
        由于机器人是按照 1D 顺序逐个点执行轨迹，因此直接在时间/路径序列上
        进行 1D 的滑动窗口平均，最符合机器人的运动学平滑需求。
        
        :param paths: 格式为 [{"point": [x,y,z], "normal": [nx,ny,nz]}, ...]
        :return: 平滑法向后的路径点集合
        """
        if not paths or len(paths) < self.window_size:
            return paths
            
        normals = np.array([p["normal"] for p in paths])
        
        # 使用 edge 模式进行边缘填充，防止首尾两端的法向在平滑时向 0 收缩
        pad_width = self.window_size // 2
        padded_normals = np.pad(normals, ((pad_width, pad_width), (0, 0)), mode='edge')
        
        smoothed_normals = np.zeros_like(normals)
        kernel = np.ones(self.window_size) / self.window_size
        
        # 对 X, Y, Z 三个分量分别进行一维卷积 (移动平均)
        for i in range(3):
            smoothed_normals[:, i] = np.convolve(padded_normals[:, i], kernel, mode='valid')
            
        # 平滑后的向量长度可能会缩短（不为 1），必须重新归一化
        norms = np.linalg.norm(smoothed_normals, axis=1)
        # 防止除以 0 的极小概率事件
        norms[norms < 1e-6] = 1.0
        smoothed_normals = smoothed_normals / norms[:, np.newaxis]
        
        # 组装返回结果（保留原有的所有字段，如 is_jump，只替换 normal）
        smoothed_paths = []
        for i, p in enumerate(paths):
            new_p = dict(p)
            new_p["normal"] = smoothed_normals[i].tolist()
            smoothed_paths.append(new_p)
            
        return smoothed_paths
