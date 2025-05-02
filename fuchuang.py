import matplotlib.pyplot as plt
from matplotlib import rcParams

# 设置中文字体
rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体（SimHei）
rcParams['axes.unicode_minus'] = False   # 解决负号显示问题

# 数据
years = [2021, 2022, 2023, 2024]
market_size = [5300, 5600, 5800, 6090]

# 创建图形
plt.figure(figsize=(8, 6))

# 绘制柱状图
plt.bar(years, market_size, color='green', alpha=0.5, label='市场规模')

# 绘制折线图
plt.plot(years, market_size, color='blue', marker='o', linestyle='-', linewidth=2, label='趋势线')

# 设置横坐标显示年份
plt.xticks(years, [str(year) for year in years], fontsize=12)

# 设置纵坐标范围
plt.ylim(0, 7000)

# 添加标题和标签
plt.title('中国农业机械市场规模', fontsize=16)
plt.xlabel('年份', fontsize=12)
plt.ylabel('市场规模（单位：亿元）', fontsize=12)

# 添加图例
plt.legend(fontsize=12)

# 显示网格
plt.grid(axis='y', linestyle='--', alpha=0.7)

# 显示图形
plt.show()


# 数据
years = [2020, 2021, 2022, 2023, 2024]
values = [1.8, 2.4, 2.5, 2.6, 2.8]

# 创建图形
plt.figure(figsize=(8, 6))

# 绘制柱状图
plt.bar(years, values, color='orange', alpha=0.7, label='数据值')

# 绘制折线图
plt.plot(years, values, color='red', marker='o', linestyle='-', linewidth=2, label='趋势线')

# 设置横坐标显示年份
plt.xticks(years, [str(year) for year in years], fontsize=12)

# 设置纵坐标范围
plt.ylim(0, 3.0)

# 添加标题和标签
plt.title('2020-2024年中国灌溉农机保有量', fontsize=16)
plt.xlabel('年份', fontsize=12)
plt.ylabel('亿台（单位）', fontsize=12)

# 添加数据点标注
for i, value in enumerate(values):
    plt.text(years[i], value + 0.05, str(value), ha='center', fontsize=10, color='black')

# 添加图例
plt.legend(fontsize=12)

# 显示网格
plt.grid(axis='y', linestyle='--', alpha=0.7)

# 显示图形
plt.show()


