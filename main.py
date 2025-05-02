from kinematic_2wheel_model import Vehicle, VehicleInfo, draw_vehicle
from kinematicsMPC import MPCController
from my_path import Path
import numpy as np
import matplotlib.pyplot as plt
import imageio.v2 as imageio
import sys
from io import BytesIO

MAX_SIMULATION_TIME = 200.0  # 程序最大运行时间200*dt

def main():
    # 设置跟踪轨迹
    path = Path(csv_file_path='./world_points.csv')  # 指定 CSV 文件路径
    rx, ry, rv, ref_yaw, ref_s, ref_kappa = path.get_ref_line_info()
    ref_path = np.column_stack((rx, ry, rv, ref_yaw, ref_s, ref_kappa))

    # 初始化车辆（假设初始位置为 (5, 60)，初始左右轮速度均为 2 m/s）
    vehicle = Vehicle(x=-5.0,
                      y=100.0,
                      yaw=0.0,
                      v_r=2.0,
                      v_l=2.0,
                      dt=0.1,
                      l=VehicleInfo.L)

    time = 0.0  # 初始时间
    target_ind = 0  # 跟踪的目标点索引
    trajectory_x = []  # 记录车辆轨迹 x
    trajectory_y = []  # 记录车辆轨迹 y
    lat_err = []  # 记录横向误差

    images = []  # 存储图片
    plt.figure(1)

    last_idx = ref_path.shape[0] - 1  # 跟踪轨迹的最后一个点的索引
    while MAX_SIMULATION_TIME >= time and last_idx > target_ind:
        time += vehicle.dt  # 累加一次时间周期

        # 调用 MPC 控制器，获取左右轮速度
        v_r, v_l, target_ind, e_y = MPCController(vehicle, ref_path)
        if v_r is None or v_l is None:
            print("An error occurred, exit...")
            sys.exit(1)

        # 记录横向误差
        lat_err.append(e_y)

        # 更新车辆状态
        vehicle.update(v_r, v_l)
        trajectory_x.append(vehicle.x)
        trajectory_y.append(vehicle.y)

        # 动态绘图
        plt.cla()
        plt.plot(ref_path[:, 0], ref_path[:, 1], '-.b', linewidth=1.0, label="Reference Path")
        draw_vehicle(vehicle.x, vehicle.y, vehicle.yaw, plt)

        plt.plot(trajectory_x, trajectory_y, "-r", label="Trajectory")
        plt.plot(ref_path[target_ind, 0], ref_path[target_ind, 1], "go", label="Target Point")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        # 将当前帧保存到内存
        buf = BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)
        images.append(imageio.imread(buf))
        buf.close()

        plt.pause(0.001)

    # 合成 GIF
    imageio.mimsave('trajectory.gif', images, duration=0.1)

    # 绘制最终结果
    plt.figure(2)
    plt.subplot(2, 1, 1)
    plt.plot(ref_path[:, 0], ref_path[:, 1], '-.b', linewidth=1.0, label="Reference Path")
    plt.plot(trajectory_x, trajectory_y, 'r', label="Vehicle Trajectory")
    plt.title("Actual Tracking Effect")
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(lat_err, label="Lateral Error")
    plt.title("Lateral Error")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
