# Truck-UAV Collaborative Delivery Optimization

## 项目简介

这是一个基于**自适应大邻域搜索算法(ALNS)**的**卡车-无人机联合配送路径优化系统**。系统旨在解决带时间窗的取送货问题(PDPTW)，实现卡车与无人机的协同配送优化。

## 项目名称

**Truck-Drone Joint Delivery Route Optimization** (卡车无人机联合配送路径优化)

## 核心功能

- **卡车配送路线规划**：基于ALNS算法优化卡车配送路径
- **无人机任务调度**：无人机的发射、取货、回收任务优化
- **卡车-无人机协同**：混合配送模式下的协同优化
- **时间窗约束**：满足客户需求的时间窗口限制
- **成本优化**：最小化总成本（运输成本 + 等待成本）
- **可视化**：配送路线图的可视化展示

## 技术栈

- Python 3.x
- NumPy - 数值计算
- Pandas - 数据处理
- Matplotlib - 数据可视化

## 项目结构

```
.
├── ALNS/                     # ALNS算法模块
│   ├── main.py              # ALNS主程序
│   ├── models.py            # 数据模型
│   ├── solution.py          # 解决方案类
│   ├── alns.py              # ALNS算法实现
│   ├── operations.py        # 算子定义
│   └── data.csv             # 测试数据
│
├── PDPTW-main/              # PDPTW问题模块
│   ├── main.py             # PDPTW主程序
│   ├── pdptw.py            # 问题定义
│   ├── route.py            # 路线类
│   ├── request.py          # 请求类
│   ├── location.py         # 位置类
│   ├── alns.py             # ALNS算法
│   ├── destroy.py          # 破坏算子
│   ├── repair.py           # 修复算子
│   ├── parameters.py       # 参数配置
│   ├── solution.py         # 解决方案
│   ├── time_analysis.py   # 时间分析
│   ├── main_scenario_analysis.py  # 场景分析
│   ├── Instances/          # 测试算例
│   │   ├── rc204C16.txt
│   │   ├── r102C18.txt
│   │   ├── lrc206.txt
│   │   └── ...
│   └── README.md           # PDPTW模块说明
│
├── generate_data.py         # 测试数据生成器
├── README.md                # 项目说明文档
└── requirements.txt         # 依赖包
```

## 使用方法

### 安装依赖

```bash
pip install numpy pandas matplotlib
```

### 运行主程序

```bash
python main.py
```

### 生成测试数据

```bash
python generate_data.py
```

## 算法说明

### ALNS (Adaptive Large Neighborhood Search)

ALNS是一种元启发式算法，通过组合不同的"破坏"和"修复"算子来搜索最优解：

- **破坏算子**：随机移除部分客户点
- **修复算子**：重新插入被移除的客户点
- **自适应机制**：根据算子表现动态调整权重

### 成本函数

总成本 = 卡车运输成本 + 无人机运输成本 + 卡车等待费用 + 无人机等待费用

## 许可证

MIT License
