# alns.py

import random
import numpy as np
import copy
import matplotlib.pyplot as plt
import time

from solution import Solution
from models import PDPTW, Route, Drone, DroneTask
from operations import Destroy, Repair


class ALNS:
    """
    ALNS
    """

    def __init__(self, problem, nDestroyOps, nRepairOps, nIterations, minSizeNBH, maxPercentageNHB, tau, coolingRate,
                 decayParameter, noise):

        self.problem = problem
        self.nDestroyOps = nDestroyOps
        self.nRepairOps = nRepairOps
        self.randomGen = random.Random(Parameters.randomSeed)

        self.wDestroy = [1 for _ in range(nDestroyOps)]  # 破坏操作的权重
        self.wRepair = [1 for _ in range(nRepairOps)]  # 修复操作的权重
        self.destroyUseTimes = [0 for _ in range(nDestroyOps)]  # 破坏操作的使用次数
        self.repairUseTimes = [0 for _ in range(nRepairOps)]  # 修复操作的使用次数
        self.destroyScore = [1 for _ in range(nDestroyOps)]  # 破坏操作的得分
        self.repairScore = [1 for _ in range(nRepairOps)]  # 修复操作的得分


        self.nIterations = nIterations
        self.minSizeNBH = minSizeNBH
        self.maxPercentageNHB = maxPercentageNHB
        self.tau = tau
        self.coolingRate = coolingRate
        self.decayParameter = decayParameter
        self.noise = noise


        self.register_weights_over_time = False
        self.removal_weights_per_iteration = []
        self.insertion_weights_per_iteration = []

        self.register_objective_value_over_time = True  # 设置为 True 以记录目标值
        self.list_objective_values = []

        self.bestSolution = None
        self.bestDistance = float('inf')
        self.bestCost = float('inf')

        self.currentSolution = None

    def printWeight(self):
        print('wDestroy:', self.wDestroy)
        print('wRepair:', self.wRepair)
        print('Destroy score:', self.destroyScore)
        print('Repair score:', self.repairScore)

    def constructInitialSolution(self):
        """
        构建初始解决方案，使用插入法生成初始路径。
        """
        print("构建初始解决方案...")
        # Initialize sets
        served_by_truck = set()
        served_by_drone = set()
        notServed = self.problem.requests.copy()

        # Create initial truck route, starting and ending at depot
        initial_truck_route = Route([self.problem.depot, self.problem.depot], set(), self.problem)

        # Create solution instance
        self.currentSolution = Solution(self.problem, initial_truck_route, served_by_truck, served_by_drone, notServed)
        self.bestSolution = self.currentSolution.copy()
        self.bestDistance = self.currentSolution.distance
        self.bestCost = self.currentSolution.total_cost

        print(f"初始解决方案的总成本: {self.bestCost:.2f} , 总距离: {self.bestDistance:.2f} km, 总时间: {self.currentSolution.time:.2f} ")

        # Compute maximum neighborhood size
        number_of_request = len(self.problem.requests)
        self.maxSizeNBH = max(1, int(np.floor(self.maxPercentageNHB / 100 * number_of_request)))
        print(f"最大邻域大小 (maxSizeNBH): {self.maxSizeNBH}")


        self.T = self.findStartingTemperature(self.tau, self.bestDistance)
        print(f"初始温度 T = {self.T}")

    def findStartingTemperature(self, startTempControlParam, starting_distance):
        """
        计算初始温度，基于设定的参数和初始距离。
        """
        delta = startTempControlParam * starting_distance
        T = float(-delta / np.log(0.5))
        return round(T, 4)

    def execute(self):
        """
        执行 ALNS 算法。
        """
        starttime = time.time()

        if self.currentSolution is None:
            self.constructInitialSolution()
        else:
            self.bestSolution = self.currentSolution.copy()
            self.bestDistance = self.currentSolution.distance
            self.bestCost = self.currentSolution.total_cost

            # 计算最大邻域大小
            number_of_request = len(self.problem.requests)
            self.maxSizeNBH = max(1, int(np.floor(self.maxPercentageNHB / 100 * number_of_request)))
            print(f"最大邻域大小 (maxSizeNBH): {self.maxSizeNBH}")

            # 计算初始温度
            self.T = self.findStartingTemperature(self.tau, self.bestDistance)
            print(f"初始温度 T = {self.T}")

        print(f"初始解决方案的总成本: {self.bestCost:.2f} , 总距离: {self.bestDistance:.2f} km, 总时间: {self.bestSolution.time:.2f} ")

        # 开始 ALNS 迭代
        for i in range(self.nIterations):
            # 选择破坏和修复操作
            destroyOpNr = self.determineDestroyOpNr()
            repairOpNr = self.determineRepairOpNr()
            sizeNBH = self.randomGen.randint(self.minSizeNBH, self.maxSizeNBH)

            # 执行破坏操作
            destroy = Destroy(self.problem, self.currentSolution)
            if destroyOpNr == 0:
                destroy.executeRandomRemoval(sizeNBH, self.randomGen)
            elif destroyOpNr == 1:
                # 假设第二个破坏算子是移除距离最远的请求
                destroy.executeWorstCostRemoval(sizeNBH // 2)  # 示例：移除一半
            elif destroyOpNr == 2:
                # 第三个破坏算子是群集破坏算子
                destroy.executeClusterRemoval(sizeNBH, self.randomGen)

            # 执行修复操作
            repair = Repair(self.problem, self.currentSolution)
            if repairOpNr == 0:
                repair.executeRandomInsertion(self.randomGen)
            elif repairOpNr == 1:
                repair.executeGreedyInsertion()
            elif repairOpNr == 2:
                repair.executeNearestNeighborInsertion()


            repaired_cost = self.currentSolution.total_cost

            improvement = False
            # 如果新的解决方案更优，接受并更新最佳解
            if repaired_cost < self.bestSolution.total_cost:
                improvement = True
                self.bestSolution = self.currentSolution.copy()
                self.bestDistance = self.currentSolution.distance
                self.bestCost = self.currentSolution.total_cost
                print(f"找到更优解: 总成本={self.bestCost:.2f} , 无人机任务数={len(self.bestSolution.drone_tasks)}",self.bestDistance)
                self.destroyScore[destroyOpNr] += Parameters.w1
                self.repairScore[repairOpNr] += Parameters.w1
            else:
                # 根据接受概率决定是否接受较差的解决方案
                if self.T > 0:
                    acceptance_prob = np.exp(- (repaired_cost - self.bestSolution.total_cost) / self.T)
                else:
                    acceptance_prob = 0
                rand_val = self.randomGen.random()
                if rand_val < acceptance_prob:
                    improvement = False
                    self.bestSolution = self.currentSolution.copy()
                    print(f"接受较差的解: 总成本={repaired_cost:.2f} USD, 接受概率={acceptance_prob:.4f}, 随机值={rand_val:.4f}")
                    self.destroyScore[destroyOpNr] += Parameters.w3
                    self.repairScore[repairOpNr] += Parameters.w3
                else:
                    self.destroyScore[destroyOpNr] += Parameters.w4
                    self.repairScore[repairOpNr] += Parameters.w4

            # 更新权重
            self.updateWeights(destroyOpNr, repairOpNr, improvement)
            # 降温
            self.T *= self.coolingRate

            # 记录迭代成本
            self.list_objective_values.append(self.currentSolution.total_cost)


            print(
                f'迭代 {i + 1}/{self.nIterations}, 当前总成本: {self.currentSolution.total_cost:.2f} , '
                f'总时间: {self.currentSolution.time:.2f} hours, 最佳总成本: {self.bestSolution.total_cost:.2f} ')

        endtime = time.time()
        cpuTime = round(endtime - starttime, 3)

        print(f"算法终止。最佳总成本: {self.bestSolution.total_cost:.2f} , 总时间: {self.bestSolution.time:.2f} hours, CPU 时间: {cpuTime} 秒")

        if self.register_objective_value_over_time:
            iterations_list = np.arange(1, self.nIterations + 1)
            plt.figure(figsize=(10, 5))
            plt.plot(iterations_list, self.list_objective_values, 'g-', marker='o')
            plt.xlabel('迭代次数', fontsize=12)
            plt.ylabel('总成本 (USD)', fontsize=12)
            plt.title('ALNS 迭代过程中的总成本变化')
            plt.grid(True)
            plt.show()

    def updateWeights(self, destroyOpNr, repairOpNr, improvement):
        """
        更新破坏和修复操作的权重。
        """
        self.destroyUseTimes[destroyOpNr] += 1
        self.repairUseTimes[repairOpNr] += 1

        if improvement:
            self.wDestroy[destroyOpNr] = self.wDestroy[destroyOpNr] * (1 - self.decayParameter) + \
                                         self.decayParameter * self.destroyScore[destroyOpNr]
            self.wRepair[repairOpNr] = self.wRepair[repairOpNr] * (1 - self.decayParameter) + \
                                       self.decayParameter * self.repairScore[repairOpNr]
        else:
            self.wDestroy[destroyOpNr] = self.wDestroy[destroyOpNr] * (1 - self.decayParameter) + \
                                         self.decayParameter * (self.destroyScore[destroyOpNr] * 0.5)
            self.wRepair[repairOpNr] = self.wRepair[repairOpNr] * (1 - self.decayParameter) + \
                                       self.decayParameter * (self.repairScore[repairOpNr] * 0.5)

    def determineDestroyOpNr(self):
        """
        根据破坏操作的权重选择一个破坏操作。
        """
        destroyRoulette = np.array(self.wDestroy).cumsum()
        r = self.randomGen.uniform(0, destroyRoulette[-1])
        for i, threshold in enumerate(destroyRoulette):
            if r <= threshold:
                return i
        return len(self.wDestroy) - 1

    def determineRepairOpNr(self):
        """
        根据修复操作的权重选择一个修复操作。
        """
        repairRoulette = np.array(self.wRepair).cumsum()
        r = self.randomGen.uniform(0, repairRoulette[-1])
        for i, threshold in enumerate(repairRoulette):
            if r <= threshold:
                return i
        return len(self.wRepair) - 1


class Parameters:
    """
    参数
    """
    randomSeed = 42  # 随机种子
    w1 = 1.5  # 如果新的解决方案是全局最优
    w2 = 1.2  # 如果新的解决方案比当前的更好
    w3 = 0.8  # 如果新的解决方案被接受
    w4 = 0.6  # 如果新的解决方案被拒绝
