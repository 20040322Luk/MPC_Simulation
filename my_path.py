import math
import numpy as np
import csv
from scipy.interpolate import splprep, splev


class Path:
    def __init__(self, csv_file_path='world_coordinates.csv'):
        """
        初始化 Path 类，生成参考轨迹信息
        """
        # 从 CSV 文件中读取世界坐标点
        world_coordinates = self.read_world_coordinates_from_csv(csv_file_path)
        self.ref_line = self.design_reference_line(world_coordinates)
        self.ref_yaw = self.cal_yaw()
        self.ref_s = self.cal_accumulated_s()
        self.ref_kappa = self.cal_kappa()

    def read_world_coordinates_from_csv(self, file_path):
        """
        从 CSV 文件中读取世界坐标系点 (x, y)，忽略第三列 (z)

        参数:
            file_path: CSV 文件路径

        返回:
            world_coordinates: 世界坐标点列表 [(x1, y1), (x2, y2), ...]
        """
        world_coordinates = []
        try:
            with open(file_path, 'r') as csv_file:
                reader = csv.reader(csv_file)
                next(reader)  # 跳过第一行（表头）
                for row in reader:
                    x = float(row[0])  # 第一列为 x 坐标
                    y = float(row[1])  # 第二列为 y 坐标
                    world_coordinates.append((x, y))
        except FileNotFoundError:
            print(f"文件 {file_path} 未找到，请检查路径！")
        except Exception as e:
            print(f"读取文件时出错：{e}")
        return world_coordinates

    def design_reference_line(self, world_coordinates):
        """
        使用样条插值生成参考轨迹

        参数:
            world_coordinates: 世界坐标系中的点列表 [(x1, y1), (x2, y2), ...]

        返回:
            ref_line: 包含 x, y 和 v 的参考线轨迹
        """
        # 提取 x 和 y 坐标
        x_coords = [point[0] for point in world_coordinates]
        y_coords = [point[1] for point in world_coordinates]

        # 使用 B 样条拟合路径
        tck, _ = splprep([x_coords, y_coords], s=1)  # s=0 表示尽量通过所有点
        u = np.linspace(0, 1, 1000)  # 插值点数量
        rx, ry = splev(u, tck)

        # 生成固定速度 v (单位: m/s)
        rv = np.full(rx.shape, 2)

        # 返回轨迹点 (x, y, v)
        return np.column_stack((rx, ry, rv))

    def cal_yaw(self):
        """
        计算参考线每个点的偏航角
        """
        yaw = []
        for i in range(len(self.ref_line)):
            if i == 0:
                yaw.append(math.atan2(self.ref_line[i + 1, 1] - self.ref_line[i, 1],
                                      self.ref_line[i + 1, 0] - self.ref_line[i, 0]))
            elif i == len(self.ref_line) - 1:
                yaw.append(math.atan2(self.ref_line[i, 1] - self.ref_line[i - 1, 1],
                                      self.ref_line[i, 0] - self.ref_line[i - 1, 0]))
            else:
                yaw.append(math.atan2(self.ref_line[i + 1, 1] - self.ref_line[i - 1, 1],
                                      self.ref_line[i + 1, 0] - self.ref_line[i - 1, 0]))
        return yaw

    def cal_accumulated_s(self):
        """
        计算参考线每个点的累计路径长度
        """
        s = []
        for i in range(len(self.ref_line)):
            if i == 0:
                s.append(0.0)
            else:
                s.append(s[-1] + math.sqrt((self.ref_line[i, 0] - self.ref_line[i - 1, 0]) ** 2 +
                                           (self.ref_line[i, 1] - self.ref_line[i - 1, 1]) ** 2))
        return s

    def cal_kappa(self):
        """
        计算参考线每个点的曲率
        """
        # 计算曲线各点的切向量
        dp = np.gradient(self.ref_line.T, axis=1)
        # 计算曲线各点的二阶导数
        d2p = np.gradient(dp, axis=1)
        # 计算曲率
        kappa = (d2p[0] * dp[1] - d2p[1] * dp[0]) / ((dp[0] ** 2 + dp[1] ** 2) ** (3 / 2))

        return kappa

    def get_ref_line_info(self):
        """
        获取参考线的全部信息
        """
        return self.ref_line[:, 0], self.ref_line[:, 1], self.ref_line[:, 2], self.ref_yaw, self.ref_s, self.ref_kappa
