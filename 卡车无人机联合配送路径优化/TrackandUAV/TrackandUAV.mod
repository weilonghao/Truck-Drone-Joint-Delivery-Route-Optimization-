/*********************************************
 * OPL 12.6.2.0 Model
 * Author: ks-computer
 * Creation Date: 2025-1-12 at 下午6:48:40
 *********************************************/
int n=10;                 // 客户点数量
int n_U=1;               // 无人机数量
float C1=1;              // 卡车单位距离成本
float C2=0.5;              // 无人机单位距离成本
float alpha=1;           // 卡车单位时间等待成本
float beta=1;            // 无人机单位时间等待成本
float d[1..n+1][1..n+1]=...;  // 卡车的行驶距离矩阵
float d_u[1..n+1][1..n+1]=...; // 无人机的飞行距离矩阵
float L=1000;               // 无人机最大续航里程
float S_T=5;             // 卡车服务所需时间
float S_D=3;             // 无人机服务所需时间
float V_T=60;             // 卡车速度
float V_D=80;             // 无人机速度
float S_L=0.5;               // 发射无人机所需时间
float S_R=0.5;               // 回收无人机所需时间
int M = 100;         // 大常数
float tau[1..n+1][1..n+1]; // 卡车时间矩阵
float tau_u[1..n+1][1..n+1][1..n_U]; // 无人机时间矩阵, 1..U是无人机的编号


dvar boolean x[0..n+1][0..n+1];        // 0或1，表示卡车是否从点i到点j
dvar boolean y[0..n][1..n][1..n+1][1..n_U];  // 0或1，表示无人机u是否从点i服务j并回收k
dvar float+ t[1..n+1];                  // 卡车到达每个客户点的时间
dvar float+ l[0..n];                  // 卡车离开每个客户点的时间
dvar float+ t_u[1..n+1][1..n_U];            // 无人机到达每个客户点的时间
dvar float+ l_u[1..n][1..n_U];            // 无人机离开每个客户点的时间
dvar float W[1..n+1];                   // 卡车在每个客户点的等待时间
dvar float W_u[1..n][1..n_U];           // 无人机u在每个客户点的等待时间
dvar int mu[1..n+1];                    // 用来记录客户的访问顺序
dvar boolean P[0..n+1][1..n+1];      // 卡车是否连续访问客户点 i 到客户点 j

execute time
{
	cplex.tilim = 1800;
}

minimize
  C1 * sum(i in 1..n)sum(j in 1..n+1) (d[i][j] * x[i][j]) +
  C2 * sum(i in 1..n)sum(j in 1..n:j!=i)sum(k in 1..n+1:k!=i)sum(u in 1..n_U) (d_u[i][j] + d_u[i][k] )* y[i][j][k][u] +
  alpha * sum(i in 1..n) W[i] + 
  sum(i in 1..n)sum(u in 1..n_U)beta * W_u[i][u];

subject to {
  //ct1:卡车必须从仓库出发
  sum(j in 1..n+1) x[0][j] == 1;  

  //ct2:卡车必须返回仓库
  sum(i in 1..n) x[i][n+1] == 1;
}

subject to {
  // ct3:每个客户点j，卡车流量守恒
  forall(j in 1..n) {
    sum(i in 1..n: i != j) x[i][j] == sum(k in 1..n+1: k != j) x[j][k];
  }
}

subject to {
  // ct4:每个无人机在同一节点只能发射一次
  forall(i in 1..n, u in 1..n_U) {
    sum(j in 1..n:j != i)sum(k in 1..n+1:k != i) y[i][j][k][u]<= 1;
  }
}
subject to {
  // ct5:每个无人机在同一节点只能回收一次
  forall(k in 1..n+1, u in 1..n_U) {
    sum(i in 1..n:i != k)sum( j in 1..n:j != k) y[i][j][k][u] <= 1;
  }
}

subject to {
  // ct6：每个客户点只能由卡车或无人机服务一次
  forall(j in 1..n) {
    sum(i in 1..n:i!=j) x[i][j] + sum(i in 1..n:i!=j) sum( k in 1..n+1)sum(u in 1..n_U) (y[i][j][k][u]) == 1;
  }
}
subject to {
  // ct7:确保无人机u从节点i发射并在k回收时，卡车径路经过k和i。
 forall(i in 1..n, j in 1..n: j != i, k in 1..n+1, u in 1..n_U) {
    2 * y[i][j][k][u] <= sum(h in 1..n: h != i) x[h][i] +sum(l in 1..n: l != k) x[l][k];
}
  // ct8：确保无人机从仓库0出发，服务客户点j，在k点回收，卡车经过k。
  forall(j in 1..n, k in 1..n+1, u in 1..n_U) {
    y[0][j][k][u] <= sum(h in 1..n: h != j) x[h][k];
  }
}
subject to {
  //计算卡车时间矩阵
  forall(i in 1..n+1) {
    forall(j in 1..n+1: i != j) {
      tau[i][j] == d[i][j] / V_T;  // 卡车从i到j的时间
        }
      }  
  // ct9:时间下界约束
  forall(i in 1..n, k in 1..n+1: k != i) {
    t[k] >= l[i] + tau[i][k] - M * (1 - x[i][k]);
  }
  
  // ct10:时间上界约束
  forall(i in 1..n, k in 1..n+1: k != i) {
    t[k] <= l[i] + tau[i][k] + M * (1 - x[i][k]);
  }
  // ct11：无人机离开时间与卡车到达时间关系
  forall(i in 1..n, u in 1..n_U) {
    l_u[i][u] >= t[i] - M * (1 - sum(j in 1..n: j != i) sum(k in 1..n+1: k != j && k != i) y[i][j][k][u]);
  }

  // ct12：卡车离开时间与无人机离开时间关系
  forall(i in 1..n, u in 1..n_U) {
    l[i] >= l_u[i][u] - M * (1 - sum(j in 1..n: j != i) sum(k in 1..n+1: k != j && k != i) y[i][j][k][u]);
  }
        
  //计算无人机时间矩阵
  forall(i in 1..n+1) {
    forall(j in 1..n+1: i != j) {
       forall(u in 1..n_U) {
          tau_u[i][j][u] == d_u[i][j]/ V_D;  // 无人机u从i到j的飞行时间
            }
        }
      }        
  // ct13：无人机最早到达时间
  forall(i in 1..n, j in 1..n: j != i, u in 1..n_U) {
    t_u[j][u] >= l[i] + tau_u[i][j][u] - M * (1 - sum(k in 1..n+1: k != i && k != j) y[i][j][k][u]);
  }

  // ct14：无人机最晚到达时间
  forall(i in 1..n, j in 1..n: j != i, u in 1..n_U) {
    t_u[j][u] <= l[i] + tau_u[i][j][u] + M * (1 - sum(k in 1..n+1: k != i && k != j) y[i][j][k][u]);
  }
  // ct15：无人机最早服务完成时间
  forall(j in 1..n, u in 1..n_U) {
    l_u[j][u] >= t_u[j][u] + S_D - M * (1 - sum(i in 1..n: i != j)sum(k in 1..n+1: k != i && k != j) y[i][j][k][u]);
  }
  
  // ct16：无人机最晚服务完成时间
  forall(j in 1..n, u in 1..n_U) {
    l_u[j][u] <= t_u[j][u] + S_D + M * (1 - sum(i in 1..n: i != j)sum(k in 1..n+1: k != i && k != j) y[i][j][k][u]);
  }
  // ct17：无人机从节点 j 到节点 k 的最早开始时间
  forall(j in 1..n, k in 1..n+1: k != j, u in 1..n_U) {
    t_u[k][u] >= l_u[j][u] +  tau_u[j][k][u] - M * (1 - sum(i in 1..n: i != j&&i!=k) y[i][j][k][u]);
  }

  // ct18：无人机从节点 j 到节点 k 的最晚开始时间
  forall(j in 1..n, k in 1..n+1: k != j, u in 1..n_U) {
    t_u[k][u] <= l_u[j][u] + tau_u[j][k][u] + M * (1 - sum(i in 1..n: i != j&&i!=k) y[i][j][k][u]);
  }
  // ct19：回收节点的离开时间与回收时间之间的关系
  forall(k in 1..n, u in 1..n_U) {
    l_u[k][u] >= t_u[k][u] + S_L * sum(l in 1..n: l != k) sum(m in 1..n+1:m!=l&&m!=k) y[k][l][m][u] + S_R * sum(i in 1..n:i!=k) sum(j in 1..n) y[i][j][k][u] - M * (1 - sum(i in 1..n:i!=k) sum(j in 1..n) y[i][j][k][u]);
  }
  // ct20：卡车离开时间约束1
  forall(k in 1..n, u in 1..n_U) {
    l[k] >= t_u[k][u] - M * (1 - sum(i in 1..n: i!= k) sum(j in 1..n: j != k && i != j) y[i][j][k][u]);
  }
  // ct21：卡车离开时间约束2
  forall(k in 1..n, u in 1..n_U) {
    l[k] >= l_u[k][u] + S_T - M * (1 - sum(l in 1..n: l != k) sum(m in 1..n+1: m != k&& m!=l) y[k][l][m][u]);
  }
  // ct22：无人机回收后才能再次发射
  forall(i in 1..n, k in 1..n+1:k!=i, v in 1..n: v != i, u in 1..n_U) {
    l_u[v][u] >= t_u[k][u] - M * (3 - 
      sum(j in 1..n: j != v) y[i][j][k][u] - 
      sum(m in 1..n: m != i && m != k && m != v) sum(n in 1..n+1: n != i && n != k) y[v][m][n][u] - P[i][v]);
  }
  // ct23：卡车k的等待时间约束
  forall(i in 1..n+1, k in 1..n+1, u in 1..n_U) {
    W[i] >= t_u[k][u] - t[k];
  }  
  // ct24：无人机u的等待时间约束
  forall(i in 1..n, k in 1..n+1, u in 1..n_U) {
    W_u[i][u] >= t[k] - t_u[k][u];
  }    
  // ct25：确保无人机的飞行距离之和不超过最大续航里程L
  forall(i in 1..n, j in 1..n: j != i, k in 1..n+1: k != i, u in 1..n_U) {
    y[i][j][k][u] * (d_u[i][j] + d_u[j][k]) <= L;
  }
  // ct26：确保无人机的顺序
  forall(i in 1..n, k in 1..n+1: k != i, u in 1..n_U) {
    mu[k] - mu[i] >= 1 - (n + 2) * (1 - sum(j in 1..n: j != i) y[i][j][k][u]);
  }
  // ct27：确保卡车访问顺序
  forall(i in 1..n, j in 1..n+1: j != i) {
    mu[i] - mu[j] + 1 <= (n + 2) * (1 - x[i][j]);
  } 
  //ct28:
  forall(i in 1..n, j in 1..n+1: i != j) {
  mu[i] - mu[j] >= 1 - (n + 2) * P[i][j];
  } 
  //ct29:
  forall(i in 1..n, j in 1..n+1: j!= i) {
  mu[i] - mu[j] <= -1 + (n + 2) * (1 - P[i][j]);
  }
  //ct30：保证从 i 到 j 和从 j 到 i 只能选择一个
  forall(i in 1..n, j in 1..n+1: i != j) {
  P[i][j] + P[j][i] == 1; 
  }
  //ct31: W_i 和 W_iu^D 都应该大于等于 0
  forall(i in 1..n+1) {
    W[i] >= 0;
  } 
  forall(i in 1..n,u in 1..n_U) {
     W_u[i][u] >= 0;
  }   
  //ct32: xij 和 Pij 必须为0或1
  forall(i in 1..n+1, j in 1..n+1: i != j) {
    x[i][j] == 0 || x[i][j] == 1;
    P[i][j] == 0 || P[i][j] == 1;   
  }
  // ct33：y_ijk^u 必须为 0 或 1，并且 i ≠ j ≠ k
  forall(i in 1..n, j in 1..n, k in 1..n+1: i != j && j != k && i != k, u in 1..n_U) {
    y[i][j][k][u] == 0 || y[i][j][k][u] == 1;
  }
  // ct34：t_i, t_iu^D, l_i, l_iu^D 必须大于等于 0
  forall(i in 1..n) {
    t[i] >= 0;
    l[i] >= 0;
  }  
  forall(i in 1..n, u in 1..n_U) {  
    t_u[i][u] >= 0;
    l_u[i][u] >= 0;
  }
  // μ_i 的值必须在 0 到 n+1 之间
  forall(i in 1..n+1) {
    mu[i] >= 0;
    mu[i] <= n + 1;  
  }
   // 卡车从配送中心出发到达每个节点 j
  forall(j in 1..n+1) {
  P[0][j] == 1; 
  } 
}  


execute
{
  // 输出求解目标值
  writeln("目标值: ", cplex.getObjValue());
  
  // 输出求解时间
  writeln("求解时间: ", cplex.getMIPTime(), " 秒");
}
execute
{
    // 获取目标值和求解时间
    var objValue = cplex.getObjValue();  // 获取目标值
    var mipTime = cplex.getMIPTime();    // 获取求解时间
    
    // 将目标值写入 sheet2!A2
    SheetWrite(sheet, "sheet2!A2", objValue);
    
    // 将求解时间写入 sheet2!B2
    SheetWrite(sheet, "sheet2!B2", mipTime);
}
