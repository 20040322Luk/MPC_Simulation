import matplotlib.pyplot as plt
from matplotlib import rcParams

# 数据
years = [2019, 2020, 2021, 2022, 2023, 2024]
production = [450.5, 471.7, 495.2, 521, 539.8, 545.2]

# 设置中文字体
rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体（SimHei）
rcParams['axes.unicode_minus'] = False   # 解决负号显示问题

# 计算增长率（增速）
growth_rate = []  # 去掉第一年的增速
for i in range(1, len(production)):
    rate = ((production[i] - production[i - 1]) / production[i - 1]) * 100
    growth_rate.append(round(rate, 2))

# 去掉第一年对应的年份
growth_years = years[1:]

# 创建图形
fig, ax1 = plt.subplots(figsize=(10, 6))

# 绘制柱状图（产量）
bar_width = 0.6
ax1.bar(years, production, color='#4A90E2', alpha=0.9, width=bar_width, label='中药材产量（万吨）')
ax1.set_xlabel('年份', fontsize=12)
ax1.set_ylabel('中药材产量（万吨）', fontsize=12)
ax1.set_ylim(400, 600)
ax1.set_xticks(years)
ax1.set_xticklabels(years, fontsize=10)
ax1.tick_params(axis='y', labelsize=10)

# 添加数据标签（产量）
for i, value in enumerate(production):
    ax1.text(years[i], value + 3, f'{value}', ha='center', va='bottom', fontsize=10, color='black')

# 创建第二个y轴（增长率）
ax2 = ax1.twinx()
ax2.plot(growth_years, growth_rate, color='#F5A623', marker='o', linestyle='-', linewidth=2, label='增速（%）')
ax2.set_ylabel('增速（%）', fontsize=12)
ax2.set_ylim(0, 15)  # 调整增速的Y轴范围

ax2.tick_params(axis='y', labelsize=10)

# 添加数据标签（增长率）
for i, rate in enumerate(growth_rate):
    ax2.text(growth_years[i], rate + 0.2, f'{rate}%', ha='center', va='bottom', fontsize=10, color='#F5A623')

# 添加图例
ax1.legend(loc='upper left', fontsize=10, frameon=False)
ax2.legend(loc='upper right', fontsize=10, frameon=False)

# 添加标题
plt.title('2019-2024年我国中药材产量及增速', fontsize=16, pad=20)

# 显示网格
ax1.grid(axis='y', linestyle='--', alpha=0.7)

# 调整布局
plt.tight_layout()

# 显示图形
plt.show()
