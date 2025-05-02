import math
import numpy as np


class Vehicle:
    def __init__(self,
                 x=0.0,
                 y=0.0,
                 yaw=0.0,
                 v_r=0.0,
                 v_l=0.0,
                 dt=0.1,
                 l=3.0):
        """
        二轮差速车辆模型
        :param x: 初始位置 x
        :param y: 初始位置 y
        :param yaw: 初始朝向角度 θ
        :param v_r: 初始右轮速度
        :param v_l: 初始左轮速度
        :param dt: 时间步长
        :param l: 两轮之间的距离（轴距）
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v_r = v_r
        self.v_l = v_l
        self.dt = dt
        self.L = l  # 轴距

    def update(self, v_r, v_l):
        """
        更新车辆状态
        :param v_r: 右轮速度
        :param v_l: 左轮速度
        """
        self.v_r = v_r
        self.v_l = v_l

        # 计算线速度和角速度
        v = (self.v_r + self.v_l) / 2.0  # 线速度
        omega = (self.v_r - self.v_l) / self.L  # 角速度

        # 更新状态
        self.x = self.x + v * math.cos(self.yaw) * self.dt
        self.y = self.y + v * math.sin(self.yaw) * self.dt
        self.yaw = self.yaw + omega * self.dt


class VehicleInfo:
    # Vehicle parameter
    L = 2.0  # 两轮之间的距离（轴距）
    W = 2.0  # 宽度
    LF = 2.8  # 后轴中心到车头距离
    LB = 0.8  # 后轴中心到车尾距离
    TR = 0.5  # 轮子半径
    TW = 0.5  # 轮子宽度
    WD = W  # 轮距
    LENGTH = LB + LF  # 车辆长度


def draw_vehicle(x, y, yaw, ax, vehicle_info=VehicleInfo, color='black'):
    """
    绘制车辆
    :param x: 车辆位置 x
    :param y: 车辆位置 y
    :param yaw: 车辆朝向角度 θ
    :param ax: 绘图对象
    :param vehicle_info: 车辆信息
    :param color: 颜色
    """
    vehicle_outline = np.array(
        [[-vehicle_info.LB, vehicle_info.LF, vehicle_info.LF, -vehicle_info.LB, -vehicle_info.LB],
         [vehicle_info.W / 2, vehicle_info.W / 2, -vehicle_info.W / 2, -vehicle_info.W / 2, vehicle_info.W / 2]])

    wheel = np.array([[-vehicle_info.TR, vehicle_info.TR, vehicle_info.TR, -vehicle_info.TR, -vehicle_info.TR],
                      [vehicle_info.TW / 2, vehicle_info.TW / 2, -vehicle_info.TW / 2, -vehicle_info.TW / 2,
                       vehicle_info.TW / 2]])

    rr_wheel = wheel.copy()  # 右后轮
    rl_wheel = wheel.copy()  # 左后轮
    fr_wheel = wheel.copy()  # 右前轮
    fl_wheel = wheel.copy()  # 左前轮
    rr_wheel[1, :] += vehicle_info.WD / 2
    rl_wheel[1, :] -= vehicle_info.WD / 2

    # yaw旋转矩阵
    rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                    [np.sin(yaw), np.cos(yaw)]])
    fr_wheel += np.array([[vehicle_info.L], [-vehicle_info.WD / 2]])
    fl_wheel += np.array([[vehicle_info.L], [vehicle_info.WD / 2]])

    fr_wheel = np.dot(rot, fr_wheel)
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    fl_wheel = np.dot(rot, fl_wheel)
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rr_wheel = np.dot(rot, rr_wheel)
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    rl_wheel = np.dot(rot, rl_wheel)
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y
    vehicle_outline = np.dot(rot, vehicle_outline)
    vehicle_outline[0, :] += x
    vehicle_outline[1, :] += y

    ax.plot(fr_wheel[0, :], fr_wheel[1, :], color)
    ax.plot(rr_wheel[0, :], rr_wheel[1, :], color)
    ax.plot(fl_wheel[0, :], fl_wheel[1, :], color)
    ax.plot(rl_wheel[0, :], rl_wheel[1, :], color)

    ax.plot(vehicle_outline[0, :], vehicle_outline[1, :], color)
    ax.axis('equal')


def update_ABMatrix(vehicle):
    """
    计算离散线性车辆运动学模型状态矩阵A和输入矩阵B
    return: A, B
    """
    v = (vehicle.v_r + vehicle.v_l) / 2.0  # 线速度
    omega = (vehicle.v_r - vehicle.v_l) / vehicle.L  # 角速度

    A = np.matrix([
        [1.0, 0.0, -v * vehicle.dt * math.sin(vehicle.yaw)],
        [0.0, 1.0, v * vehicle.dt * math.cos(vehicle.yaw)],
        [0.0, 0.0, 1.0]])

    B = np.matrix([
        [vehicle.dt * math.cos(vehicle.yaw) / 2.0, vehicle.dt * math.cos(vehicle.yaw) / 2.0],
        [vehicle.dt * math.sin(vehicle.yaw) / 2.0, vehicle.dt * math.sin(vehicle.yaw) / 2.0],
        [vehicle.dt / vehicle.L, -vehicle.dt / vehicle.L]])
    return A, B
