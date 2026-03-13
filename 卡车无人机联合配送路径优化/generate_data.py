import pandas as pd
import numpy as np
import math


def generate_loop_instance(num_customers, filename):
    """
    生成环路型 PDPTW 算例并保存为 CSV 文件。

    Parameters:
    - num_customers (int): 客户数量。
    - filename (str): 输出 CSV 文件名。
    """
    data = []

    # 仓库节点
    depot = {
        'CUST NO.': 0,
        'XCOORD.': 100.0,
        'YCOORD.': 0.0,
        'READY TIME': 0.0,
        'DUE DATE': 24.0,
        'DEMAND': 0,
        'SERVICE TIME': 0.0
    }
    data.append(depot)

    # 计算角度增量
    angle_increment = 360 / num_customers
    for i in range(1, num_customers + 1):
        angle_deg = 20 + angle_increment * (i - 1)  # 起始角度20度
        angle_rad = math.radians(angle_deg)
        x = 50 + 50 * math.cos(angle_rad)
        y = 0 + 50 * math.sin(angle_rad)
        ready_time = round(np.random.uniform(8.0, 16.0), 1)
        due_date = round(np.random.uniform(ready_time + 1.0, 24.0), 1)
        demand = np.random.randint(2, 8)  # 2到7公斤
        service_time = 0.5

        customer = {
            'CUST NO.': i,
            'XCOORD.': round(x, 2),
            'YCOORD.': round(y, 2),
            'READY TIME': ready_time,
            'DUE DATE': due_date,
            'DEMAND': demand,
            'SERVICE TIME': service_time
        }
        data.append(customer)

    df = pd.DataFrame(data)
    df.to_csv(filename, index=False)
    print(f"算例已生成并保存为 {filename}")


if __name__ == "__main__":
    # 生成50个客户节点的算例
    generate_loop_instance(50, 'c50.csv')

    # 生成100个客户节点的算例
    generate_loop_instance(100, 'c100.csv')
