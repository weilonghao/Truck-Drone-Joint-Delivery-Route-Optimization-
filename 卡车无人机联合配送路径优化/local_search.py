# local_search.py

import logging
from models import Route


class LocalSearch:

    def __init__(self, solution):
        self.solution = solution

    def two_opt(self):
        improved = True
        while improved:
            improved = False
            route = self.solution.truck_route.locations
            best_distance = self.solution.truck_route.distance
            for i in range(1, len(route) - 2):
                for j in range(i + 1, len(route) - 1):
                    if j - i == 1:
                        continue
                    new_route = route[:i] + route[i:j][::-1] + route[j:]
                    temp_route = Route(locations=new_route, requests=self.solution.served_by_truck.copy(), problem=self.solution.problem)
                    if temp_route.distance < best_distance and temp_route.is_feasible():
                        self.solution.truck_route = temp_route
                        self.solution.distance = temp_route.distance
                        self.solution.time = max(self.solution.time, self.solution.truck_route.start_service_times[-1] + self.solution.truck_route.service_time)
                        self.solution.computeTotalCost()
                        improved = True
                        best_distance = temp_route.distance
                        logging.info("应用2-opt优化卡车路线，新的距离: %.2f km", best_distance)
            if not improved:
                break

    def three_opt(self):
        improved = True
        while improved:
            improved = False
            route = self.solution.truck_route.locations
            best_distance = self.solution.truck_route.distance
            n = len(route)
            for i in range(1, n - 2):
                for j in range(i + 1, n - 1):
                    for k in range(j + 1, n):
                        new_route = self.three_opt_swap(route, i, j, k)
                        temp_route = Route(locations=new_route, requests=self.solution.served_by_truck.copy(), problem=self.solution.problem)
                        if temp_route.distance < best_distance and temp_route.is_feasible():
                            self.solution.truck_route = temp_route
                            self.solution.distance = temp_route.distance
                            self.solution.time = max(self.solution.time, self.solution.truck_route.start_service_times[-1] + self.solution.truck_route.service_time)
                            self.solution.computeTotalCost()
                            improved = True
                            best_distance = temp_route.distance
                            logging.info("应用3-opt优化卡车路线，新的距离: %.2f km", best_distance)
            if not improved:
                break

    def three_opt_swap(self, route, i, j, k):
        """
        实现3-opt交换的所有可能方式之一，这里仅示例一种方式。
        """
        # 这里可以实现更多的交换方式以进一步优化
        new_route = route[:i] + route[j:k] + route[i:j] + route[k:]
        return new_route
