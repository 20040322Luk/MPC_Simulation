import numpy as np
import math
import cvxpy as cp
from kinematic_2wheel_model import VehicleInfo, update_ABMatrix


# 系统配置
NX = 3  # 状态向量的个数: x = [x, y, yaw]
NU = 2  # 输入向量的个数: u = [v_r, v_l] (左右轮速度)
NP = 10  # 有限时间视界长度：预测过程中考虑的时间范围的有限长度
MAX_V = 35  # 最大车速(m/s)
MAX_DELTA_V = 10.0  # 设置最大速度差值（可根据需求调整）

# MPC config
Q = np.diag([2.0, 4.0, 2.0])  # 运行状态代价
F = np.diag([2.0, 2.0, 2.0])  # 末端状态代价
R = np.diag([0.01, 0.01])  # 输入状态代价


def calc_preparation(vehicle, ref_path):
    """
    计算参考轨迹的 xref、uref、index 和横向误差 er
    """
    rx, ry, rv, ryaw, rkappa = ref_path[:, 0], ref_path[:, 1], ref_path[:, 2], ref_path[:, 3], ref_path[:, 5]
    dx = [vehicle.x - icx for icx in rx]
    dy = [vehicle.y - icy for icy in ry]
    d = np.hypot(dx, dy)
    index = np.argmin(d)  # 找到最近的参考点索引

    # 计算横向误差 er
    vec_nr = np.array([math.cos(ryaw[index] + math.pi / 2.0),
                       math.sin(ryaw[index] + math.pi / 2.0)])
    vec_target_2_rear = np.array([vehicle.x - rx[index],
                                  vehicle.y - ry[index]])
    er = np.dot(vec_target_2_rear, vec_nr)

    # 生成参考状态和输入
    xref = np.zeros((NX, NP + 1))
    uref = np.zeros((NU, NP + 1))

    for i in range(NP + 1):
        ind = min(index + i, len(rx) - 1)
        xref[0, i] = rx[ind]
        xref[1, i] = ry[ind]
        xref[2, i] = ryaw[ind]

        # 参考速度和输入
        uref[0, i] = rv[ind]  # 右轮速度
        uref[1, i] = rv[ind]  # 左轮速度（假设左右轮速度相同）

    return xref, uref, index, er


def MPCController(vehicle, ref_path):
    """
    使用 MPC 控制器计算左右轮速度 (v_r, v_l)
    """
    # 1. 准备参考轨迹数据
    xref, uref, index, er = calc_preparation(vehicle, ref_path)

    # 初始状态
    x0 = [vehicle.x, vehicle.y, vehicle.yaw]

    # 定义优化变量
    x = cp.Variable((NX, NP + 1))  # 状态变量 [x, y, yaw]
    u = cp.Variable((NU, NP))  # 输入变量 [v_r, v_l]

    # 初始化代价函数和约束条件
    cost = 0.0
    constraints = []

    # 初始状态约束
    constraints += [x[:, 0] == x0]

    for i in range(NP):
        # 运行代价
        cost += cp.quad_form(u[:, i] - uref[:, i], R)
        if i != 0:
            cost += cp.quad_form(x[:, i] - xref[:, i], Q)

        # 获取线性化的系统矩阵 A 和 B
        A, B = update_ABMatrix(vehicle)

        # 动力学约束
        constraints += [
            x[:, i + 1] - xref[:, i + 1] == A @ (x[:, i] - xref[:, i]) + B @ (u[:, i] - uref[:, i])
        ]

    # 末端状态代价
    cost += cp.quad_form(x[:, NP] - xref[:, NP], F)

    # 输入约束（左右轮速度限制）
    constraints += [cp.abs(u[0, :]) <= MAX_V]  # v_r
    constraints += [cp.abs(u[1, :]) <= MAX_V]  # v_l
    # **增加左右轮速度差值的约束**

    constraints += [cp.abs(u[0, :] - u[1, :]) <= MAX_DELTA_V]

    # 定义优化问题
    problem = cp.Problem(cp.Minimize(cost), constraints)

    # 求解优化问题
    problem.solve(solver=cp.ECOS, verbose=False)

    # 检查求解状态
    if problem.status == cp.OPTIMAL or problem.status == cp.OPTIMAL_INACCURATE:
        # 提取最优控制输入
        opt_u = u.value[:, 0]
        v_r = opt_u[0]  # 最优右轮速度
        v_l = opt_u[1]  # 最优左轮速度
        return v_r, v_l, index, er
    else:
        print("Error: MPC solution failed!")
        # 如果求解失败，返回当前速度（保持不变）
        return vehicle.v_r, vehicle.v_l, index, er


# 示例使用
if __name__ == "__main__":
    class Vehicle:
        def __init__(self, x, y, yaw, v_r, v_l):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v_r = v_r
            self.v_l = v_l
            self.L = VehicleInfo.L

        def update(self, v_r, v_l, dt=0.1):
            """
            使用左右轮速度更新车辆位置
            """
            v = (v_r + v_l) / 2.0
            omega = (v_r - v_l) / self.L
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += omega * dt

    # 初始化车辆
    vehicle = Vehicle(x=0.0, y=0.0, yaw=0.0, v_r=0.0, v_l=0.0)

    # 生成参考路径 (示例)
    ref_path = np.zeros((100, 6))
    ref_path[:, 0] = np.linspace(0, 50, 100)  # x
    ref_path[:, 1] = np.linspace(0, 0, 100)  # y
    ref_path[:, 2] = 5.0  # 速度
    ref_path[:, 3] = 0.0  # 偏航角
    ref_path[:, 5] = 0.0  # 曲率

    # 调用 MPC 控制器
    v_r, v_l, target_ind, e_y = MPCController(vehicle, ref_path)
    print(f"Optimal v_r: {v_r}, v_l: {v_l}, Target Index: {target_ind}, Lateral Error: {e_y}")
