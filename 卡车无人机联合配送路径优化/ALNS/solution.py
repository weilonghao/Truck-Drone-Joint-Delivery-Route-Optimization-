# solution.py

from models import Route, DroneTask, Drone, Request
import copy


class Solution:
    def __init__(self, problem, truck_route, served_by_truck, served_by_drone, notServed):
        self.problem = problem
        self.truck_route = truck_route  # Route 对象
        self.served_by_truck = served_by_truck  # Set of Request objects
        self.served_by_drone = served_by_drone  # Set of Request objects
        self.notServed = notServed  # List of Request objects
        self.truck_waiting_cost = None
        self.drone_waiting_cost = None
        # 初始化无人机任务列表
        self.drone_tasks = []

        # 初始化无人机状态
        self.drone_states = []
        for drone in problem.drones:
            self.drone_states.append({
                'drone': copy.deepcopy(drone),
                'current_range': drone.max_range,
                'available_time': 0.0
            })

        self.distance, self.time = self.computeDistanceAndTime()

        self.total_cost = self.computeTotalCost()

    def computeDistanceAndTime(self):
        total_distance = self.truck_route.distance
        total_time = self.truck_route.start_service_times[-1] + self.truck_route.service_time

        for task in self.drone_tasks:
            launch_node = task.launch_node
            pickup_node = task.pickup_node
            recovery_node = task.recovery_node
            drone_id = task.drone_id
            distance_launch_to_pickup = self.problem.distMatrix[launch_node][pickup_node]
            distance_pickup_to_recovery = self.problem.distMatrix[pickup_node][recovery_node]
            total_distance += distance_launch_to_pickup + distance_pickup_to_recovery

            drone_state = next((d for d in self.drone_states if d['drone'].drone_id == drone_id), None)
            if drone_state:
                task_completion_time = task.launch_time + (distance_launch_to_pickup + distance_pickup_to_recovery) / drone_state['drone'].speed + self.problem.drone_launch_recovery_time
                total_time = max(total_time, task_completion_time)
            else:
                print(f"Warning: Drone {drone_id} not found in solution.drone_states.")

        return total_distance, total_time

    def computeTotalCost(self):

        total_cost = 0.0

        truck_transport_cost = self.truck_route.distance * self.problem.truck_transport_cost
        total_cost += truck_transport_cost

        drone_transport_cost = 0.0
        for task in self.drone_tasks:
            distance = self.problem.distMatrix[task.launch_node][task.pickup_node] \
                       + self.problem.distMatrix[task.pickup_node][task.recovery_node]
            drone_transport_cost += distance * self.problem.drone_transport_cost
        total_cost += drone_transport_cost

        drone_reward = 0.0
        for task in self.drone_tasks:
            drone_reward += 30
        total_cost -= drone_reward

        truck_waiting_cost = 0.0
        drone_waiting_cost = 0.0

        node_to_arrival_time = {loc.nodeID: arrival_time for loc, arrival_time in
                                zip(self.truck_route.locations, self.truck_route.arrival_times)}

        for task in self.drone_tasks:
            recovery_node = task.recovery_node
            recovery_time = task.recovery_time



            truck_arrival_time = node_to_arrival_time.get(recovery_node, 0.0)

            waiting_time = truck_arrival_time - recovery_time

            if waiting_time > 0:
                drone_waiting_cost += abs(waiting_time)*self.problem.drone_waiting_cost
            else:
                truck_waiting_cost += abs(waiting_time)*self.problem.truck_waiting_cost


        total_cost += truck_waiting_cost
        total_cost += drone_waiting_cost

        self.truck_waiting_cost = truck_waiting_cost
        self.drone_waiting_cost = drone_waiting_cost

        return total_cost

    def removeRequest(self, request):


        if request in self.served_by_truck:
            self.truck_route.removeRequest(request)
            self.served_by_truck.discard(request)

            if len(self.truck_route.requests) == 0:
                self.truck_route.locations = [self.problem.depot, self.problem.depot]
                self.truck_route.distance = 0.0
                self.truck_route.service_time = 0.0
                self.truck_route.arrival_times = [0.0, 0.0]
                self.truck_route.start_service_times = [0.0, 0.0]

        tasks_to_remove = [task for task in self.drone_tasks if task.pickup_node == request.deliveryLoc.nodeID]
        for task in tasks_to_remove:
            self.drone_tasks.remove(task)
            self.served_by_drone.discard(request)
            drone_state = next((d for d in self.drone_states if d['drone'].drone_id == task.drone_id), None)
            if drone_state:
                drone_state['current_range'] += (self.problem.distMatrix[task.launch_node][task.pickup_node] +
                                                self.problem.distMatrix[task.pickup_node][task.recovery_node])

                drone_state['available_time'] = 0.0
            else:
                print(f"Warning: Drone {task.drone_id} not found in solution.drone_states.")

        self.notServed.append(request)

        self.distance, self.time = self.computeDistanceAndTime()
        self.total_cost = self.computeTotalCost()

    def optimize_truck_route(self):

        self.truck_route.optimize_route()

        self.distance, self.time = self.computeDistanceAndTime()
        self.total_cost = self.computeTotalCost()

    def insert_into_truck(self, request):

        if request in self.served_by_drone:

            return False

        inserted = self.truck_route.greedyInsert(request)
        if inserted and inserted[0]:
            # 更新卡车路线和相关参数
            self.truck_route = inserted[0]
            self.distance += inserted[1]
            self.time = max(self.time, self.truck_route.start_service_times[-1] + self.truck_route.service_time)
            self.total_cost = self.computeTotalCost()
            self.served_by_truck.add(request)
            self.notServed.remove(request)


            self.truck_route.optimize_route()
            return True
        else:

            return False

    def assign_to_drone(self, request):

        if request in self.served_by_truck:

            return False

        # 遍历卡车路线的每一段（从节点i到节点i+1）
        for i in range(len(self.truck_route.locations) - 1):
            launch_node = self.truck_route.locations[i].nodeID
            recovery_node = self.truck_route.locations[i + 1].nodeID

            # 如果发射节点和回收节点相同，跳过
            if launch_node == recovery_node:

                continue

            distance_launch_to_pickup = self.problem.distMatrix[launch_node][request.deliveryLoc.nodeID]
            if distance_launch_to_pickup > 15.0:
                continue

            for drone_state in self.drone_states:
                drone = drone_state['drone']
                current_range = drone_state['current_range']
                available_time = drone_state['available_time']

                pickup_node = request.deliveryLoc.nodeID

                if launch_node == pickup_node:
                    print(f"无人机任务发射节点与配送节点相同: {launch_node} == {pickup_node}")
                    continue

                if recovery_node == pickup_node:
                    print(f"无人机任务回收节点与配送节点相同: {recovery_node} == {pickup_node}")
                    continue

                distance_pickup_to_recovery = self.problem.distMatrix[pickup_node][recovery_node]
                total_distance = distance_launch_to_pickup + distance_pickup_to_recovery


                if total_distance > current_range:
                    continue
                if request.demand > drone.capacity:

                    continue

                truck_launch_time = self.truck_route.start_service_times[i]


                flight_time = (distance_launch_to_pickup + distance_pickup_to_recovery) / drone.speed
                task_launch_time = truck_launch_time  # 无人机发射时间与卡车同步
                task_recovery_time = task_launch_time + flight_time + self.problem.drone_launch_recovery_time

                if task_launch_time < drone_state['available_time']:
                    continue


                if any(task.pickup_node == pickup_node for task in self.drone_tasks):
                    continue


                task = DroneTask(
                    launch_node=launch_node,
                    pickup_node=pickup_node,
                    recovery_node=recovery_node,
                    drone_id=drone.drone_id,
                    launch_time=task_launch_time,
                    recovery_time=task_recovery_time
                )

                self.drone_tasks.append(task)
                self.served_by_drone.add(request)
                self.notServed.remove(request)

                # 更新无人机的续航里程和可用时间
                drone_state['current_range'] -= total_distance
                drone_state['available_time'] = task_recovery_time


                # 更新总距离和时间
                self.distance += distance_launch_to_pickup + distance_pickup_to_recovery
                self.time = max(self.time, task_recovery_time)
                self.total_cost = self.computeTotalCost()

                return True

    def copy(self):

        truck_route_copy = self.truck_route.copy() if self.truck_route else None

        served_by_truck_copy = self.served_by_truck.copy()
        served_by_drone_copy = self.served_by_drone.copy()
        notServed_copy = self.notServed.copy()

        copy_solution = Solution(
            problem=self.problem,
            truck_route=truck_route_copy,
            served_by_truck=served_by_truck_copy,
            served_by_drone=served_by_drone_copy,
            notServed=notServed_copy
        )

        copy_solution.drone_tasks = copy.deepcopy(self.drone_tasks)


        copy_solution.drone_states = copy.deepcopy(self.drone_states)

        copy_solution.distance, copy_solution.time = copy_solution.computeDistanceAndTime()
        copy_solution.total_cost = copy_solution.computeTotalCost()

        copy_solution.truck_route.optimize_route()

        return copy_solution