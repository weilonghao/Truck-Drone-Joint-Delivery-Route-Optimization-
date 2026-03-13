# main.py

import pandas as pd
import matplotlib.pyplot as plt
from models import Location, Request, PDPTW, Route, Drone, DroneTask
from solution import Solution
from alns import ALNS, Parameters
import numpy as np
import random
import logging

np.random.seed(42)
random.seed(42)

def main():

    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # 读取数据文件

    df = pd.read_csv('data.csv')


    depot_row = df.iloc[0]
    depot = Location(
        nodeID=int(depot_row['CUST NO.']),
        typeLoc=0,  # 0: Depot
        x=float(depot_row['XCOORD.']),
        y=float(depot_row['YCOORD.']),
        demand=int(depot_row['DEMAND']),
        requestID=None  # 仓库没有请求ID
    )


    locations = {depot.nodeID: depot}
    requests = []

    existing_request_ids = set()

    for idx, row in df.iloc[1:].iterrows():
        nodeID = int(row['CUST NO.'])
        if nodeID in existing_request_ids:
            logging.warning(f"请求 ID {nodeID} 已存在，跳过重复请求。")
            continue

        x = float(row['XCOORD.'])
        y = float(row['YCOORD.'])
        demand = int(row['DEMAND'])

        deliveryLoc = Location(
            nodeID=nodeID,
            typeLoc=2,
            x=x,
            y=y,
            demand=demand,
            requestID=nodeID,
            service_time=float(row.get('SERVICE TIME', 0.5))
        )

        locations[nodeID] = deliveryLoc

        request = Request(
            ID=nodeID,
            pickUpLoc=depot,
            deliveryLoc=deliveryLoc,
            demand=demand
        )
        requests.append(request)
        existing_request_ids.add(nodeID)

    drones = [
        Drone(drone_id=1, max_range=10000, speed=80, capacity=1),
    ]

    node_ids = list(locations.keys())
    distMatrix = {i: {} for i in node_ids}
    for i in node_ids:
        for j in node_ids:
            if i == j:
                distMatrix[i][j] = 0.0
            else:
                xi, yi = locations[i].x, locations[i].y
                xj, yj = locations[j].x, locations[j].y
                distance = np.sqrt((xi - xj) ** 2 + (yi - yj) ** 2)
                distMatrix[i][j] = distance


    problem = PDPTW(
        distMatrix=distMatrix,
        depot=depot,
        requests=requests,
        drones=drones,
        drone_launch_recovery_time=0.5,
        truck_transport_cost=5,
        drone_transport_cost=1,
        truck_waiting_cost=1,
        drone_waiting_cost=1,
        truck_speed=60.0
    )

    served_by_truck = set()
    served_by_drone = set()
    notServed = requests.copy()

    initial_truck_route = Route([depot, depot], set(), problem)

    # 创建解决方案实例
    initial_solution = Solution(
        problem=problem,
        truck_route=initial_truck_route,
        served_by_truck=served_by_truck,
        served_by_drone=served_by_drone,
        notServed=notServed
    )

    logging.info("\n开始构建初始解决方案...")
    while initial_solution.notServed:
        req = random.choice(initial_solution.notServed)
        if random.random() < 0.5:
            assigned = initial_solution.assign_to_drone(req)
            if assigned:
                logging.info(f"客户点 {req.ID} 分配给无人机。")
                continue
        inserted = initial_solution.insert_into_truck(req)
        if inserted:
            logging.info(f"客户点 {req.ID} 插入到卡车路径。")
            continue
        else:
            assigned = initial_solution.assign_to_drone(req)
            if assigned:
                logging.info(f"客户点 {req.ID} 分配给无人机。")
                continue
            else:
                logging.warning(f"客户点 {req.ID} 无法被服务。")
                initial_solution.notServed.remove(req)  # 移除未能服务的请求

    logging.info(f"\n初始解决方案完成。已服务客户数: {len(initial_solution.served_by_truck)} (卡车), "
                 f"{len(initial_solution.served_by_drone)} (无人机), 未服务客户数: {len(initial_solution.notServed)}")

    # 初始化 ALNS 参数
    nDestroyOps = 3
    nRepairOps = 3
    nIterations = 500
    minSizeNBH = 1
    maxPercentageNHB = 5
    tau = 0.03
    coolingRate = 0.999
    decayParameter = 0.15
    noise = 0.015

    # 初始化 ALNS 实例
    alns = ALNS(
        problem=problem,
        nDestroyOps=nDestroyOps,
        nRepairOps=nRepairOps,
        nIterations=nIterations,
        minSizeNBH=minSizeNBH,
        maxPercentageNHB=maxPercentageNHB,
        tau=tau,
        coolingRate=coolingRate,
        decayParameter=decayParameter,
        noise=noise
    )

    alns.currentSolution = initial_solution.copy()
    alns.bestSolution = initial_solution.copy()
    alns.bestDistance = initial_solution.distance
    alns.bestCost = initial_solution.total_cost

    number_of_request = len(problem.requests)
    alns.maxSizeNBH = max(1, int(np.floor(alns.maxPercentageNHB / 100 * number_of_request)))
    logging.info(f"最大邻域大小 (maxSizeNBH): {alns.maxSizeNBH}")

    alns.T = alns.findStartingTemperature(alns.tau, alns.bestDistance)
    logging.info(f"初始温度 T = {alns.T}")

    logging.info(f"\n初始解决方案的总成本: {alns.bestCost:.2f} , 总距离: {alns.bestDistance:.2f} km, 总时间: {initial_solution.time:.2f} hours")

    # 运行 ALNS 算法
    alns.execute()

    best_sol = alns.bestSolution
    best_cost = best_sol.total_cost
    logging.info(best_sol)
    truck_waiting_cost = best_sol.truck_waiting_cost
    drone_waiting_cost = best_sol.drone_waiting_cost

    truck_transport_cost = best_sol.truck_route.distance * problem.truck_transport_cost
    drone_transport_cost = sum(
        (problem.distMatrix[task.launch_node][task.pickup_node] +
         problem.distMatrix[task.pickup_node][task.recovery_node]) * problem.drone_transport_cost
        for task in best_sol.drone_tasks
    )

    # 计算无人机服务节点数
    drone_service_nodes = len(best_sol.drone_tasks)

    # 打印结果
    logging.info("\n===== 最终结果 =====")
    logging.info(f"最佳总成本: {best_cost:.2f} ")
    logging.info(f" - 卡车运输成本: {truck_transport_cost:.2f} ")
    logging.info(f" - 无人机运输成本: {drone_transport_cost:.2f} ")
    logging.info(f" - 卡车等待费用: {truck_waiting_cost:.2f} ")
    logging.info(f" - 无人机等待费用: {drone_waiting_cost:.2f} ")
    logging.info(f" - 无人机服务节点数: {drone_service_nodes}")

    logging.info("\n最佳卡车路线:")
    truck_route_nodes = [loc.nodeID for loc in best_sol.truck_route.locations]
    logging.info(f"路线: {truck_route_nodes}")


    # 验证无人机任务的有效性
    for task in best_sol.drone_tasks:
        if task.launch_node == task.pickup_node == task.recovery_node:
            logging.warning(f"无人机 {task.drone_id} 的任务无效，因为发射、配送和回收节点相同。")

    locations_dict = locations

    plt.figure(figsize=(12, 12))

    truck_x = [locations_dict[node].x for node in truck_route_nodes]
    truck_y = [locations_dict[node].y for node in truck_route_nodes]
    plt.plot(truck_x, truck_y, 'b-', marker='o', label='卡车路线')

    if best_sol.drone_tasks:
        for idx, task in enumerate(best_sol.drone_tasks):
            launch_loc = locations_dict[task.launch_node]
            pickup_loc = locations_dict[task.pickup_node]
            recovery_loc = locations_dict[task.recovery_node]
            #if task.launch_node == task.pickup_node and task.pickup_node == task.recovery_node:
            #    plt.plot(launch_loc.x, launch_loc.y, 'r*', markersize=10, label='无人机任务' if idx == 0 else "")
            plt.plot([launch_loc.x, pickup_loc.x, recovery_loc.x], [launch_loc.y, pickup_loc.y, recovery_loc.y],
                         'r--o', label=f'无人机任务 {task.drone_id}' if idx == 0 else "")
    else:
        logging.info("无无人机任务需要绘制。")

    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())

    for node, loc in locations_dict.items():
        plt.text(loc.x, loc.y, str(node), fontsize=8, ha='right')

    plt.title("配送路线图")
    plt.xlabel("X 坐标")
    plt.ylabel("Y 坐标")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
