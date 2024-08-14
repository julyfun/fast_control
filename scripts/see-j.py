# input full ros2 log, get lines with 'joints:', and plot the joints
import matplotlib.pyplot as plt
# 打开文件
import sys
with open(sys.argv[1], 'r') as file:
    # 逐行读取文件
    data = []
    for line in file:
        # 检查当前行是否包含 'joints:'
        if 'joints:' in line:
            # 打印包含 'joints:' 的行
            l = line.strip()
            w = l.split(' ')
            nums = w[-6:]
            nums = list(map(float, nums))
            t = float(w[1][1:-2])
            data.append([t, nums])
    print(len(data))
    st = data[0][0]
    for d in data:
        d[0] -= st
    fig, ax = plt.subplots()
    lt = st
    for i, sublist in enumerate(data):
        if sublist[0] - lt > 0.1:
            print(sublist[0] + st)
        for j, value in enumerate(sublist[1]):
            ax.plot(sublist[0], value, 'bo')  # 'bo' 是蓝色圆点的意思
        lt = sublist[0]
    
    ax.set_title('Plot of Lists')
    ax.set_xlabel('Index of List')
    ax.set_ylabel('Values')
    
    plt.show()

