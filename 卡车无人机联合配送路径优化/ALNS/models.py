# models.py

import copy
import numpy as np
import logging


class Location:
    def __init__(self, nodeID, typeLoc, x, y, demand=0, requestID=None, service_time=0.5):

        self.nodeID = nodeID
        self.typeLoc = typeLoc
        self.x = x
        self.y = y
        self.demand = demand
        self.requestID = requestID
        self.service_time = 0
        self.arrival_time = 0.0
        self.start_service_time = 0.0


class Request:
    def __init__(self, ID, pickUpLoc, deliveryLoc, demand):

        self.ID = ID
        self.pickUpLoc = pickUpLoc  # Location 对象
        self.deliveryLoc = deliveryLoc  # Location 对象
        self.demand = demand

    def __eq__(self, other):
        if isinstance(other, Request):
            return self.ID == other.ID
        return False

    def __hash__(self):
        return hash(self.ID)


class Drone:
    def __init__(self, drone_id, max_range, speed, capacity):

        self.drone_id = drone_id
        self.max_range = max_range
        self.speed = speed
        self.capacity = capacity
        self.current_range = max_range
        self.available_time = 0.0


class DroneTask:
    def __init__(self, launch_node, pickup_node, recovery_node, drone_id, launch_time, recovery_time):

        self.launch_node = launch_node
        self.pickup_node = pickup_node
        self.recovery_node = recovery_node
        self.drone_id = drone_id
        self.launch_time = launch_time
        self.recovery_time = recovery_time

        self.waiting_time = 0.0  # 无人机等待时间


class PDPTW:
    def __init__(self, distMatrix, depot, requests, drones=None, drone_launch_recovery_time=0.5,
                 truck_transport_cost=1.0, drone_transport_cost=0.5,
                 truck_waiting_cost=10.0, drone_waiting_cost=15.0, truck_speed=60.0):

        self.distMatrix = distMatrix
        self.depot = depot
        self.requests = requests
        if drones is None:
            self.drones = []
        else:
            self.drones = drones
        self.drone_launch_recovery_time = drone_launch_recovery_time
        self.truck_transport_cost = truck_transport_cost
        self.drone_transport_cost = drone_transport_cost
        self.truck_waiting_cost = truck_waiting_cost
        self.drone_waiting_cost = drone_waiting_cost
        self.truck_speed = truck_speed


class Route:
    def __init__(self, locations, requests, problem):

        self.locations = locations
        self.requests = requests
        self.problem = problem
        self.distance = self.computeDistance()
        self.service_time = self.computeServiceTime()
        self.computeTimes()

    def computeDistance(self):

        distance = 0.0
        for i in range(1, len(self.locations)):
            from_node = self.locations[i - 1].nodeID
            to_node = self.locations[i].nodeID
            distance += self.problem.distMatrix[from_node][to_node]
        return distance

    def computeServiceTime(self):

        service_time = 0.0
        for loc in self.locations:
            if loc.typeLoc == 2:  # 配送点
                service_time += loc.service_time
        return service_time

    def computeTimes(self):

        self.arrival_times = []
        self.start_service_times = []
        current_time = 0.0
        for i in range(len(self.locations)):
            if i == 0:
                # 仓库的到达时间为0
                self.arrival_times.append(current_time)
                self.start_service_times.append(current_time)
                # 假设卡车在仓库无需服务时间
            else:
                # 计算从前一个节点到当前节点的时间
                from_node = self.locations[i - 1]
                to_node = self.locations[i]
                travel_time = self.problem.distMatrix[from_node.nodeID][to_node.nodeID] / self.problem.truck_speed
                arrival_time = current_time + travel_time
                self.arrival_times.append(arrival_time)

                # 服务开始时间
                start_service_time = arrival_time
                self.start_service_times.append(start_service_time)

                # 更新当前时间，加上服务时间
                if to_node.typeLoc == 2:  # 配送点
                    current_time = start_service_time + to_node.service_time
                else:
                    current_time = start_service_time

    def is_feasible(self):

        return True

    def removeRequest(self, request):

        new_locations = []
        for loc in self.locations:
            if loc.requestID != request.ID:
                new_locations.append(loc)
        # 确保路线始终以仓库结束
        if new_locations[-1].nodeID != self.problem.depot.nodeID:
            new_locations.append(self.problem.depot)
        self.locations = new_locations
        self.requests.discard(request)
        self.distance = self.computeDistance()
        self.service_time = self.computeServiceTime()
        self.computeTimes()

    def greedyInsert(self, request):

        best_cost = float('inf')
        best_route = None


        for i in range(1, len(self.locations)):

            new_locations = self.locations.copy()
            new_locations.insert(i, request.deliveryLoc)

            # 确保不会在插入后出现连续重复节点
            if new_locations[i - 1].nodeID == new_locations[i].nodeID:
                continue  # 跳过，避免连续重复

            # 创建临时路线
            temp_route = Route(
                locations=new_locations,
                requests=self.requests.copy().union({request}),
                problem=self.problem
            )
            additional_cost = temp_route.distance - self.distance



            if additional_cost < best_cost and temp_route.is_feasible():
                best_cost = additional_cost
                best_route = temp_route

        if best_route:
            return best_route, best_cost
        else:
            return None, None

    def optimize_route(self):

        improved = True
        while improved:
            improved = False
            best_distance = self.distance
            for i in range(1, len(self.locations) - 2):
                for j in range(i + 1, len(self.locations) - 1):
                    if j - i == 1:
                        continue
                    new_locations = self.locations[:i] + self.locations[i:j][::-1] + self.locations[j:]
                    temp_route = Route(locations=new_locations, requests=self.requests.copy(), problem=self.problem)
                    if temp_route.distance < best_distance:
                        self.locations = new_locations
                        self.distance = temp_route.distance
                        self.service_time = temp_route.service_time
                        self.computeTimes()
                        improved = True

                        break
                if improved:
                    break

    def copy(self):

        return Route(copy.deepcopy(self.locations), self.requests.copy(), self.problem)
