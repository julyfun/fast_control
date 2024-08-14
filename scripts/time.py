# input full ros2 log, show [time] interval
import sys
import matplotlib.pyplot as plt

def plot_distribution(data):
    """ 绘制实数的分布图。"""
    plt.figure(figsize=(10, 6))
    plt.hist(data, bins=200, color='blue', edgecolor='black')
    plt.title('Distribution of Numbers')
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.grid(True)
    plt.show()

with open(sys.argv[1], 'r') as file:
    lt = -1
    nums = []
    for line in file:
        w = line.split(' ')
        t = float(w[1][1:-2])
        if lt >- 0:
            if t - lt > 0.02:
                print(f'at {t}, time {t - lt}')
            nums.append(t - lt)
        lt = t
    plot_distribution(nums)

        
