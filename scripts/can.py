# input special log by j.pct
import matplotlib.pyplot as plt
import sys

def read_data(filename):
    """ 从文件中读取实数，并返回一个列表。"""
    data = []
    with open(filename, 'r') as file:
        for line in file:
            # 将读取的行转换为浮点数并添加到列表中
            try:
                nums = list(map(float, line.strip().split()))
                data += nums
            except ValueError:
                # 如果转换失败（例如，空行或非数字字符），则跳过该行
                continue
    return data

def plot_distribution(data):
    """ 绘制实数的分布图。"""
    plt.figure(figsize=(10, 6))
    plt.hist(data, bins=200, color='blue', edgecolor='black')
    plt.title('Distribution of Numbers')
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.grid(True)
    plt.show()

def main():
    # 文件路径
    filename = sys.argv[1]
    # 读取数据
    data = read_data(filename)
    data.sort()
    print(f'len: {len(data)}')
    for i in range(10, 0, -1):
        per = i * 0.1
        prop = 1 - per / 100
        print(f"{per:.1}% slow: {data[int(len(data) * prop)]}")
    print('max:', data[-1])
    # 绘制数据分布
    leng = len(data)
    max10 = ""
    for i in range(leng - 10, leng):
        max10 += str(data[i]) + " "
    print(f'max10: {max10}')
    plot_distribution(data)

if __name__ == '__main__':
    main()
