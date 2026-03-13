# operations.py

from solution import Solution
from models import DroneTask, Drone, Route, Request
import numpy as np
import random


class Destroy:
    def __init__(self, problem, solution):
        self.problem = problem
        self.solution = solution

    def executeRandomRemoval(self, size, randomGen):
        """
        随机移除一定数量的请求，并将这些请求标记为未服务。
        """
        total_served = len(self.solution.served_by_truck) + len(self.solution.served_by_drone)
        if total_served == 0:
            return
        min_requests = max(1, int(0.2 * total_served))  # 保留至少20%的请求
        removable = total_served - min_requests
        if removable <= 0:
            return
        remove_size = min(size, removable)
        combined_served = list(self.solution.served_by_truck) + list(self.solution.served_by_drone)
        removed_requests = randomGen.sample(combined_served, remove_size)

        # 存储被删除的无人机任务
        removed_drone_tasks = []

        for req in removed_requests:
            # 如果是卡车路径的客户且是无人机的发射或回收点，移除无人机任务
            for task in self.solution.drone_tasks:
                if task.pickup_node == req.deliveryLoc.nodeID or task.recovery_node == req.deliveryLoc.nodeID:
                    removed_drone_tasks.append(task)
                    self.solution.drone_tasks.remove(task)
                    break

            self.solution.removeRequest(req)
            self.solution.notServed.append(req)


        self.solution.removed_drone_tasks = removed_drone_tasks

    def executeWorstCostRemoval(self, size):
        """
        最大路径删除算子
        """
        total_served = len(self.solution.served_by_truck) + len(self.solution.served_by_drone)
        if total_served == 0:
            return
        combined_served = list(self.solution.served_by_truck) + list(self.solution.served_by_drone)
        sorted_requests = sorted(
            combined_served,
            key=lambda r: self.problem.distMatrix[self.problem.depot.nodeID][r.deliveryLoc.nodeID],
            reverse=True
        )
        min_requests = max(1, int(0.1 * total_served))
        removable_requests = sorted_requests[:max(0, len(sorted_requests) - min_requests)]
        removed_requests = removable_requests[:size]

        removed_drone_tasks = []

        for req in removed_requests:
            for task in self.solution.drone_tasks:
                if task.pickup_node == req.deliveryLoc.nodeID or task.recovery_node == req.deliveryLoc.nodeID:
                    removed_drone_tasks.append(task)
                    self.solution.drone_tasks.remove(task)
                    break

            self.solution.removeRequest(req)
            self.solution.notServed.append(req)

        self.solution.removed_drone_tasks = removed_drone_tasks

    def executeClusterRemoval(self, size, randomGen):
        """
        群集破坏算子。随机选择客户点作为焦点，移除其及最近邻的客户点
        """
        total_served = len(self.solution.served_by_truck) + len(self.solution.served_by_drone)
        if total_served == 0:
            return
        removed = 0
        removed_drone_tasks = []
        while removed < size and (len(self.solution.served_by_truck) + len(self.solution.served_by_drone)) > 0:
            combined_served = list(self.solution.served_by_truck) + list(self.solution.served_by_drone)
            focal_req = randomGen.choice(combined_served)
            self.solution.removeRequest(focal_req)
            self.solution.notServed.append(focal_req)
            removed += 1

            for task in self.solution.drone_tasks:
                if task.pickup_node == focal_req.deliveryLoc.nodeID or task.recovery_node == focal_req.deliveryLoc.nodeID:
                    removed_drone_tasks.append(task)
                    self.solution.drone_tasks.remove(task)
                    break

            distances = []
            for req in self.solution.served_by_truck.union(self.solution.served_by_drone):
                distance = self.problem.distMatrix[focal_req.deliveryLoc.nodeID][req.deliveryLoc.nodeID]
                distances.append((req, distance))
            distances.sort(key=lambda x: x[1])

            for req, dist in distances[:2]:
                if removed >= size:
                    break
                self.solution.removeRequest(req)
                self.solution.notServed.append(req)
                removed += 1

        self.solution.removed_drone_tasks = removed_drone_tasks


class Repair:
    def __init__(self, problem, solution):
        self.problem = problem
        self.solution = solution

    def executeRandomInsertion(self, randomGen):
        """
        随机插入修复算子。将未服务的请求随机插入卡车路径或分配给无人机。
        """
        if not self.solution.notServed:
            return

        randomGen.shuffle(self.solution.notServed)

        for req in self.solution.notServed.copy():
            if req in self.solution.served_by_truck or req in self.solution.served_by_drone:
                continue

            # 尝试分配给无人机
            assigned = self.solution.assign_to_drone(req)
            if assigned:
                continue

            # 如果无法分配给无人机，则尝试插入卡车路径
            inserted = self.solution.insert_into_truck(req)
            if inserted:
                continue
            else:
                assigned = self.solution.assign_to_drone(req)
                if assigned:
                    continue
                else:
                    self.solution.notServed.remove(req)

        # 将移除的无人机任务重新插入
        for task in self.solution.removed_drone_tasks:
            self.solution.drone_tasks.append(task)
        self.solution.removed_drone_tasks = []

    def executeGreedyInsertion(self):
        """
        贪婪插入修复算子
        """
        if not self.solution.notServed:
            return

        sorted_requests = sorted(self.solution.notServed, key=lambda r: r.demand, reverse=True)

        for req in sorted_requests:
            if req in self.solution.served_by_truck or req in self.solution.served_by_drone:
                continue

            assigned = self.solution.assign_to_drone(req)
            if assigned:
                continue

            inserted = self.solution.insert_into_truck(req)
            if inserted:
                continue
            else:
                assigned = self.solution.assign_to_drone(req)
                if assigned:
                    continue
                else:
                    self.solution.notServed.remove(req)

        # 将移除的无人机任务重新插入
        for task in self.solution.removed_drone_tasks:
            self.solution.drone_tasks.append(task)
        self.solution.removed_drone_tasks = []

    def executeNearestNeighborInsertion(self):
        """
        最近邻插入修复算子
        """
        if not self.solution.notServed:
            return

        shuffled_requests = self.solution.notServed.copy()
        random.shuffle(shuffled_requests)

        for req in shuffled_requests:
            if req in self.solution.served_by_truck or req in self.solution.served_by_drone:
                continue

            truck_nodes = [loc.nodeID for loc in self.solution.truck_route.locations]
            distances = [self.problem.distMatrix[node][req.deliveryLoc.nodeID] for node in truck_nodes]
            nearest_node = truck_nodes[np.argmin(distances)]

            best_increase = float('inf')
            best_position = None
            best_route = None

            for i in range(len(self.solution.truck_route.locations) - 1):
                if self.solution.truck_route.locations[i].nodeID == nearest_node:
                    new_locations = self.solution.truck_route.locations.copy()
                    new_locations.insert(i + 1, req.deliveryLoc)
                    if i + 1 > 0 and new_locations[i].nodeID == new_locations[i + 1].nodeID:
                        continue
                    temp_route = Route(locations=new_locations, requests=self.solution.served_by_truck.copy().union({req}), problem=self.problem)
                    additional_cost = temp_route.distance - self.solution.truck_route.distance
                    if additional_cost < best_increase and temp_route.is_feasible():
                        best_increase = additional_cost
                        best_position = i + 1
                        best_route = temp_route

            if best_route:
                assigned = self.solution.assign_to_drone(req)
                if assigned:
                    continue
                else:
                    self.solution.truck_route = best_route
                    self.solution.distance += best_increase
                    self.solution.time = max(self.solution.time, self.solution.truck_route.start_service_times[-1] + self.solution.truck_route.service_time)
                    self.solution.total_cost = self.solution.computeTotalCost()
                    self.solution.served_by_truck.add(req)
                    self.solution.notServed.remove(req)
            else:
                assigned = self.solution.assign_to_drone(req)
                if assigned:
                    continue
                else:
                    self.solution.notServed.remove(req)


        for task in self.solution.removed_drone_tasks:
            self.solution.drone_tasks.append(task)
        self.solution.removed_drone_tasks = []
