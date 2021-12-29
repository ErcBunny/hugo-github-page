---
title: "Notes for Introduction to Autonomous Mobile Robots"
date: 2021-12-27T15:24:52+08:00
draft: false
description: A Fall 2021 Course 「移动机器人导论」
image: /post/note/mobileRobot/cover.jpg
comments: true
license: false
hidden: false
categories:
    - note
tags:
---

---
## 绪论

### 按功能分类
* 移动机器人
* 操作机器人
* 移动作业机器人
* 举些例子：机械臂，月球车，心脏手术，扫地，UAV，自驾车

### 应用领域
* 工业
* 空间探测
* 军事
* 医疗手术
* 家庭

### 几个概念
* 自主性(autonomy)是行为主体按自己意愿行事的动机、能力或特性
    * 自主移动机器人是一类可以根据任务需求、具有在单维或多维空间主动改变自身位姿以及空间位置能力的机器人统称
    * 自动驾驶分级：0-2级需要驾驶员实时介入，自动驾驶提供辅助速度和车道控制；3级有时会要求接管驾驶，4级开始是在特定环境下完全自主，5级是所有环境下全自主
        * 0-2：特斯拉，小鹏，未来
        * 没有一个达到了3
* 三大关键问题：在哪，去哪，怎么去
* 经典3环节
    * see (perception): sensing, info extraction (filtering, keypoint extraction and matching)
    * think (planning/understanding): localizatoon, mapping, planning
    * act (motion control): tracking, actuator driving

![general pipeline](/post/note/mobileRobot/see-think-act.png)

---
## 运动形式

### 运动相关的概念
* locomotion，运动，是一种机器人与环境的物理交互
* 稳定性考虑：接触点数目和形状（角度，摩擦），重心，环境（地形，空气），本体稳定性

### 两种地面运动形式
* 腿式运动
    * 自然界偏爱，自然界表面粗糙，环境复杂
    * 通过性强，控制困难，非连续点接触
    * 效率取决于本体质量
    * 近似滚动多边形，随着步距减小，接近圆形
* 轮式运动
    * 自然界基本没有，因为关节不能旋转
    * 在平面上运动效率高，控制简单，受环境约束大
    * 效率取决于环境质量

### 腿式机器人
* 特点（因为不同场景下优点和弱点互相转换）包括：适应地形，自主调节重心高度，运动执行器也能当成操作器，主动隔离震动，自由度高，难控制建模，对关节驱动要求高，需要配合地形感知，落地有冲击
* 稳定性
    * 动态稳定指的是执行器停止工作时机器人摔倒，反之则是静态稳定的
    * 静止时保持稳定的条件
        * 点接触腿需要三支同时着地（波士顿动力方案，快，动态稳定，能耗低）
        * 面接触腿只需要一支（比如日本方案的双足，静态稳定，慢，能耗高）
    * 运动时保持稳定的条件
        * 静态步态行走需要4-6条腿
* 关节数和自由度：每个腿至少需要两个自由度，增加自由度可以提高机动性、步态稳定性，同时增加设计难度
* 可能事件总数Gait/步态：一个行进周期内腿式机器人每条腿抬起和落地可能性的组合$=(2k-1)\!$注意与腿状态数区分 $=2^k$

### 轮式机器人
* 特点：人造结构的高效性（滚动摩擦，重心不会起伏变化），结构简单，成本低，控制简单，系统复杂度低，运行速度高
* 稳定性：至少有三个车轮同时接触地面才能保证静态稳定性（重心落在接触点的三角形内部），3个轮子以上需要悬挂系统使所有轮子保持与地面接触
* 轮子类型
    * 标准轮，DOF=2
    * 脚轮，DOF=3，调向时会对机器人底盘施加一个扭矩，偏心距d:触地点到垂直旋转轴的距离
    * 瑞典轮（十字连续切换90/麦克纳姆45），DOF=3，无法单独使用，至少需三个或以上共同使用，对地面冲击大，噪音大，易破坏地面，运行震动大，对机器人本体机构冲击大
    * 球轮，DOF=3，无约束:具有很高的灵活性，成本高:制造和维护成本均很高，积灰、磨损
* 全向驱动经典布局：等边三角形，瑞典轮，转向标准轮，4麦克纳姆

---
## 轮式机器人运动学

### 非完整约束
* 大部分轮式机器人形态均含有非完整约束，即无法从位置空间找到一一对应的关系，和历史状态有关
* 需要扩展到速度空间 (车轮间相对位置增量的差别)考虑微分运动学

### 坐标系描述
* 惯性参考坐标系 $\xi_I$ ：机器人作业目标及控制指令，传感器感知测量的环境信息
* 机器人参考坐标系 $\xi_R$ ：机器人控制器的误差输入以及控制指令
* config矩阵可以是 $3 \times 3$ ：因为只有两个维度
* $\bold{\dot{\xi_I}=R(\theta)\dot{\xi_R}}$ ，两个坐标系下的速度之间关系为旋转矩阵

![坐标系的一般设置](/post/note/mobileRobot/coord.png)

### (前向)运动学模型
* 指建立惯性系下参考点速度和执行器速度之间的关系
* 中间桥梁为机器人系下参考点速度
* 构建方法
    * 作用法：从轮运动参数（转速，半径，轮距），进行速度的合成与分解（同时考虑约束作用），得到参考点在机体坐标系下的速度，然后通过旋转矩阵转为世界坐标系
    * 约束法：把每个轮子的约束方程写出来（轮运动参数与机体坐标下速度的关系），合并成一个式子并转换为世界坐标速度
* 标准轮约束
    * 主动固定标准轮：$v_{\parallel}=r \dot{\phi}$ 以及 $v_{\perp}=0$ ；将 $v_{\parallel}$ 和 $v_{\perp}$ 用机体坐标系下的速度表示出来
    * 主动转向标准轮：滚动约束，可控制角度来改变运动： $v_{\parallel}=r \dot{\phi}$
    * 随动固定标准轮：无侧滑约束（滑动约束）： $v_{\perp}=0$
    * 随动转向标准轮：自由
* 脚轮约束
    * 作为主动轮：滚动约束，可控制角度来改变运动
    * 作为从动轮：自由
* 球轮约束
    * 主动：与主动转向标准轮一致
    * 从动：自由
* 瑞典轮约束
    * 主动：驱动速度 $\dot{\phi} r$ 往转子上投影作为最终这个执行器产生的速度，没有无侧滑约束
        * 记 $\gamma$ 为滚子轴与轮平面的夹角，则滚动约束为 $\dot{\phi} r \cos(\gamma)=v_{\parallel}$ ，这里 $v_{\parallel}$ 指的是机体坐标系下速度滚子轴方向的投影
    * 从动：自由
* 零运动直线：几何上经过轮子的轴心并垂直于轮平面的线，当受无侧滑约束时，轮子在该直线上不能存在运动
    * 脚轮、瑞典轮：不存在无侧滑约束，不存在零运动直线
* 轮式移动机器人运动学建模仅考虑平面运动，因此其对应的输出状态为三维向量，因此，最多有三个独立约束
* 动力学建模在运动变化较为快速、动态响应比较明显的情况下适用，比如无人机上

![运动学模型例子](/post/note/mobileRobot/kine-example.png)

### 自由度
* 自由度是机动性的度量
* 总自由度=移动mobility自由度+操纵steerability自由度：$\delta_{M}=\delta_{m}+\delta_{s}$
    * $\delta_{M}=3$：瞬心可在平面的任意点，可在工作空间中跟踪任何路径
    * $\delta_{M}=2$：瞬心被限制在某条直线上
* 移动自由度（可移动度）$\delta_{m}$
    * $\delta_{m}=$ 工作空间维度 $-$ 独立约束数目 $=$ 状态向量的维度 $-$ 滑动约束中的独立约束个数
    * 考虑 $\bold{AR\dot{\xi_I}} = [\bold{B,0}]^T$ 这样动力学模型的基本形式，与 $\bold{0}$ 对应的 $\bold{A}$ 的部分子矩阵零空间维度 $=$ 列数 $-$ 秩 $=\delta_{m}$
    * $\delta_{m}=0$ 时无法在平面中运动， $\delta_{m}=1$ 时只能沿着圆弧/直线行走（移动性退化）， $\delta_{m}=2$ 时可原地（线速度和角速度解耦）， $\delta_{m}=3$ 时全向
        * 二轮差速小车：通过改变轮转速可同时控制角速度和线速度， $\delta_{m}=2$
        * 自行车：改变主动轮转速只能改变线速度，角速度要通过另一个轮子的方向， $\delta_{m}=1$
* 操纵自由度（可操纵度） $\delta_{s}$
    * 等于独立的能够转向的舵机的个数，范围是 $[0,2]$
    * 考虑 $\bold{A}$ 矩阵中滑动约束部分，挑出其中转向轮的子矩阵，求秩，则等于 $\delta_{s}$
* 机器人的完整性判据：可移动度等于工作空间维度
    * 对于地面移动机器人，工作空间维度是3

![自由度计算例子](/post/note/mobileRobot/dof-example.png)

---
## 二轮差速机器人的控制

> 只考虑运动学控制，仅分析二轮差速小车

### 概念和零碎知识点
* 定点(镇定)控制Regulation Control：以指定姿态到达指定工作位置
* 路径跟踪控制Path Tracking Control：跟随给定路线
* 轨迹跟踪控制Trajectory Tracking Control：跟随给定的轨迹
* 对于非完整约束机器人而言，不存在静态反馈控制律，使得机器人达到目标位姿
* 移动机器人反馈控制器设计一般步骤
    1. 根据作业需求定义系统开环误差信号
    2. 误差信号描述变换:惯性参考坐标系、机器人参考坐标系
    3. 基于机器人模型，构建闭环系统误差模型
    4. 控制器设计
    5. 稳定性分析
    6. 仿真实验
    7. 实际实验
* 自治系统(Autonomous system)：控制量只依赖状态不依赖时间
* 微分平坦(differentially flat)系统：控制量可以用状态量及其导数来表示
* 非完整约束系统总是欠驱动系统
* 可控：能通过施加控制使系统到达状态空间中的任意一个状态，是模型本身的性质，与控制量无关
* 系统存在光滑的反馈控制的必要条件，Brockett定理

### 定点控制

![机器人模型](/post/note/mobileRobot/robot2ctrl.png)

* 记号和问题描述
    * $q=[x,y,\theta]^T$ 为世界坐标系下的状态
    * $q_r=[x_r,y_r,\theta_r]^T$ 为世界坐标系下参考状态
    * $\tilde{q}=[\tilde{x},\tilde{y},\tilde{\theta}]^T=q-q_r$ 为开环误差
    * $e=\bold{R(\theta)^T} \tilde{q}$ 为机体坐标系下误差信号
    * 误差的动态模型是 $\dot{e}$ 使用“左导右不导……”可展开，矩阵求导这里是每个元素分别求导即可
    * 定点控制是找到一系列 $\bold{\dot{\xi_R}} = [v(t),\omega(t)]^T = [\dot{x}(t), \dot{y}(t), \omega(t)]^T = \bold{K}e$ 使 $e(t)=0$， 这里 $\bold{K}$ 为控制矩阵，是要设计的变量， $\bold{\dot{\xi_R}}$ 则能通过运动学模型转化为执行器需要的输出
    * 这个模型中 $\dot{y}(t)=0$ ， $\dot{x}(t)=v(t)$
* 惯性坐标系中机器人的运动学模型
    * $\bold{\dot{\xi_I}} = \bold{R(\theta)\bold{\dot{\xi_R}}}$
    * 根据此以及Brockett定理可判定对于这个模型，没有光滑的反馈控制
    * 根据Chow定理是可控的
* 非线性控制器
    * 记机体坐标系下误差信号为 $e=[e_1,e_2,e_3]^T$
    * 可令 $v(t)=-k_1 e_1$ ，单纯的比例控制
    * 可令 $\omega(t)=-k_2 e_3 + {e_2}^2\sin(t)$ ，比例控制加前馈
    * 如此带入误差的动态模型 $\dot{e}$ 可以进行稳定性分析
* 极坐标线性化
    * $\tilde{q}=[\rho, \alpha, \beta]$
    * 误差模型则是 $\dot{\tilde{q}}=\bold{A} [v,\omega]^T$
    * 控制器则可以令 $v=k_{\rho}\rho, \omega=k_1 \alpha + k_2 \beta$
    * 带入到误差模型里后还是存在三角非线性，因为 $\alpha$ 始终比较小，可以做三角函数的线性化得 $\dot{\tilde{q}} = \bold{A_{linear}} \tilde{q}$
    * 由Hurwitz判据知系统指数收敛，又近似了LTI，所以渐进收敛

![误差模型的推导](/post/note/mobileRobot/pole1.png)
![控制器和线性化](/post/note/mobileRobot/pole2.png)
![稳定性分析](/post/note/mobileRobot/pole3.png)

> 并不是说极坐标一定要线性化，也可以指定 $v,\omega$ 为其他的函数，非线性的也可以，只不过就得用Lyapnuov理论了
>
> <u>**一般方法都是得到误差状态变量的导数与 $[v,w]^T$ 的关系（误差模型），然后设计控制率，也就是 $v, w$ 的关于状态变量的函数，然后带入到误差模型中，得到误差的动态，对其做稳定性分析**</u>

### 轨迹跟踪控制
* $q_r$ 这时候是规定的一条轨迹了，类似上面的定点控制
* 更高级的在这里有[「横向自动控制方法：Purepursuit, Stanley, MPC对比」](https://blog.csdn.net/weixin_46723764/article/details/108885265)


### 路径跟踪控制
* 课程内容不具有普遍参考意义，摆了罢

---
## 规划

### 几个基本概念
* The workspace is often the representation of the world, possibly independent of the robot itself. Often describes some notion of reachability, what space is free or occupied? 不一定是笛卡尔空间，关节空间或者参数空间都可以
* Configuration space describes the full state of the robot in the world (actuator positions, orientation, etc.)
* Let’s consider that our robot is no longer a point, but occupies an area...
* structured: occupancy grid map, distance field; Unstructured: graph, mesh, exact

### 基于搜索树的路径查找
* [一个很好的视觉实现](https://qiao.github.io/PathFinding.js/visual/)
* 广度优先搜索BFS
    * 从起点开始，首先遍历起点周围邻近的点，然后再遍历已经遍历过的点邻近的点，逐步的向外扩散
    * 一旦到达终点，便可以从终点开始，反过来顺着父节点的顺序找到起点，由此就构成了一条路径
    * 完备
* 深度优先搜索DFS
    * 从起点开始，不断沿着路径进行扩展，直到找到终点或者无路可走，再重新选取父节点进行深度搜索
    * 不完备，可能无限走错路
* Edsger W. Dijkstra
    * 寻找图形中节点之间的最短路径，需要计算每一个节点距离起点的总移动代价
    * 节点放入优先队列中会按照代价进行排序，运行时优选选出代价最小的作为下一个遍历的节点
    * Asymptotically the fastest known single-source shortest path algorithm for arbitrary directed graphs
    * Doesn’t really know the goal exists until it reaches it
* 从Dijkstra到A*
    * 综合代价=代价+启发函数
    * 优先选取综合代价小的节点作为下一个路径点
    * 启发函数是节点n距离终点的预计代价
        * 估计值 $<=$ 实际代价：一定能找到最短路径，估计值越小则遍历的节点越多
        * 估计值 $=$ 实际代价：一定能找到最短路径，最理想的情况
        * 估计值 $>$ 实际代价：不能保证找到最短路径，不过此时会很快，若远大于则变为最佳优先搜索
    * 在低维度空间里很常用，限制是需要构造一个图，启发函数比较难找，比较难栅格化

![Dijkstra和A*区别](/post/note/mobileRobot/Dijk-Astar.png)

* D*: Dynamic A*
    * 动态环境下可能各种条件代价会变化所以进行了改进

### 基于采样的路径查找
* 主要是为了防止维数爆炸，防止连通性降级，以及直接在config空间里（不好grid化）直接plan
* 主要思路
    1. 在config空间中采样（uniform，gaussian，obstacle involved）
    2. 将新采样点和附近的已有点进行连接，判据一般是范数
    3. 检查碰撞（计算量大户），如果没碰撞则添加一条边到路径中
    4. 检查是否到达目标（附近）
    5. loop

![范数](/post/note/mobileRobot/norm.png)

* Probabilistic Roadmaps (PRM, 1996)
* Rapidly-exploring random trees (RRT, 1998)