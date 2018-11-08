# planning 算法和框架介绍

整个规划模块负责通过从系统中上游得到的感知信息，判断本车当前所处的场景，输出可行的连续轨迹交由下游控制器模块执行。
规划模块主要依赖于本车的精确定位、局部地图的支持和对周围静态和动态障碍物的检测以及运动估计。
在项目实现中规划模块的功能主体对应于实例化planning_engine，从其它模块得到的输入如下：

* 从 Navi 或者本车当前位置速度和姿态信息
* 从 MapReader 获取当前位置附近的道路信息
* 从 Prediction 获取 障碍物信息(包含预测)以及本车周围区Â域是否有障碍物(变道用)
* 从 Light 获取红绿灯信息

在ROS中具体对应的topic如下所示:
```c++
READ_STRING_PARAM_WITH_DEFAULT(navi, "/navi/fusion");
    READ_STRING_PARAM_WITH_DEFAULT(map, "/map/lanes");
    READ_STRING_PARAM_WITH_DEFAULT(light, "/compute/vision/light");
    READ_STRING_PARAM_WITH_DEFAULT(prediction, "/object/prediction");
    READ_STRING_PARAM_WITH_DEFAULT(fusion, "/compute/vision/car/front");
    READ_STRING_PARAM_WITH_DEFAULT(brakeInfo, "/vehicle/brake_info_report");
    READ_STRING_PARAM_WITH_DEFAULT(steeringReport, "/vehicle/steering_report");
    READ_STRING_PARAM_WITH_DEFAULT(enabled, "/vehicle/dbw_enabled");
    READ_STRING_PARAM_WITH_DEFAULT(speedLimit, "/speed_limit");
    READ_STRING_PARAM_WITH_DEFAULT(web, "/web");
```
输出主要包含期望速度和加速度、以及包含时间维度的离散点轨迹：
```c++
 planning_pubr_ = private_nh_.advertise<momenta_msgs::Planning>("/planning", 1);
 ```
## 行为规划

### 行为规划主要功能
行为规划处于整个规划模块的顶层，负责对接收的感知、定位和局部地图等信息进行综合处理，利用有限状态机(finite statemachine)的机制实时判断本车当前所处的状态。目前在基于既定全局参考路线下的实时局部规划的任务场景下，主要根据当前行进方向和路线制定了一系列的状态。主要职能是在保证整体执行路线和参考路线基本一致的情况下，实时应对cut in, merge, intersection等典型的道路场景，做出合适的变道决策和路径&速度规划，在遵循交通规则（车道和路口的速度限制）和本车运动学&动力学模型（加速度限制和尺寸等）的前提下而规避潜在的障碍物。

具体来说，Option和其派生的OptionMain负责维护整个状态机的运转。其中WorldModel接收感知和地图等外部输入，并对数据做预处理；MotionPlanner负责维护底层运动规划器的输入信息载入和调用；Actuators负责输出动作，也即发布planning消息。

### 状态定义
基本状态：
```c++
const std::string Option::STATE_INIT = "init";
const std::string Option::STATE_ABORT = "abort";
const std::string Option::STATE_FINISH = "finish";
```
定制状态：
```c++
Option(id, {"disabled", "lane_keep", "crossing", "merge"})
```
### 状态转移关系
* 接收使能信号从disable状态跳转到lane_keep状态
* 接近路口时由lane_keep状态跳转到crossing状态
* 驶离路口后从crossing状态跳转到lane_keep状态
* 汇入场景对应的merge状态有待开发

### 变道决策

变道需要依次考虑一系列因素，均满足的情况下才实施变道。依次考虑的因素如下：

1. 不可频繁变道，即距离上次成功变道的时间不能太短。
2. 目标车道必须可用，即目标车道存在且未被障碍物占据。
3. 必须有变道需求，即己车不在中间车道，或己车被前方低速障碍物阻拦，无法达到预期速度甚至有碰撞风险。
4. 前方障碍物非误检，即已经连续出现若干帧。（今后感知误检率降至一定水平后可以将该条件去掉）
5. 当前道路曲率足够小，允许安全变道。
6. 综合考虑给出的分值达到阈值。
    * 当前车道评分机制：考虑当前车道的前车速度、前车距离、前车预期碰撞时间、己车速度等因素，求其加权平均值
    * 目标车道评分机制：目前较简单，仅仅考虑目标车道是否是中心车道。
    * 综合评分机制：考虑目标车道与当前车道分数之比，结合当前路段曲率、距离下一个路口的距离，得到的分数高于一定阈值时，允许变道。

## 运动规划
此算法中的 Motion Planning 包含了 Path Planning 和 Speed Planning 两个部分，在当前 Frenet 坐标系中进行计算，最后反投影到全局笛卡尔坐标并差值得到轨迹输出。

### Frenet 坐标系
在整个planning框架中，涉及到的坐标系主要有局部坐标系、车辆坐标系和frenet坐标系。局部坐标系使用东-北-天坐标（ENU），x,y,z轴分别指向地理上的东向、北向和天顶。车辆坐标系则采用以车辆正前方为x轴，z轴朝上的右手系。
Frenet坐标系又称Frenet–Serret公式，在规划模块中被广泛使用。Frenet–Serret公式用于描述粒子在三维欧氏空间 R3
内沿一条连续可微曲线的运动学特征。关于frenet坐标系在三维空间下的详细说明可以参考https://en.wikipedia.org/wiki/Frenet–Serret_formulas。
在车辆的运动规划中，Frenet坐标系简化到二维平面，使用道路的中心线作为参考线,使用参考线的切线向量t和法线向量n建立坐标系。通常将沿着参考线的方向称为s方向，参考线当前位置的法向称为d方向。相较于Cartesian坐标系，Frenet坐标系将车体在道路上的运动规划由一个二维平面上的问题转化为两个一维方向上的问题。基于参考线的位置表示可以使用(s,d,t)也即纵向距离和横向距离以及时间描述，同样的两个方向上的速度用 $\dot{s},\dot{d},t$ 表示。
Frenet坐标系和Cartesian坐标系的位置转换可以用描述为：
$$ \textbf{x}(s(t),d(t)) = \textbf{r}(s(t)) + d(t)\textbf{n}_c(s(t))$$

![frenet coordinate frame](resource/frenet.png)

在实际使用中，需要做速度和加速度的坐标系转化，也即$s, \dot{s}, \ddot{s}, d, \dot{d}, \ddot{d}$ 和 $x,y,\theta, \kappa, v, a$相互转换。具体的转换相对繁琐，详细可参考文献[1]。
关于 Frenet 坐标系的建立以及和笛卡尔坐标系之间的变换，工具函数参见 utils/frenet_coordinate_system.h

### LATTICE 路径/速度规划
lattice算法是一种在frenet坐标系下纵向和横向解耦的路径/速度规划算法，通过对轨迹的采样、打分和筛选机制得到较优的三维轨迹（二维空间和一维时间）。其基本的目标和流程可总结为：
* 车道保持场景是以当前车道中心线作为参考规划轨迹，横向偏移 d 恒等于0
* 在变道情景下以目标车道中心线重新建立frenet坐标系，回归到车道保持场景
* 在目标车道中心线采样，终点横向偏差 ，五次多项式拟合d(s)，并对不同轨迹根据横向位置、速度和加速度进行打分，选取最优路径，确定 $d(s), \dot{d}(s), \ddot{d}(s)$

#### 算法原理
具体的算法主要由BMW的Moritz Werling提出，在Frenet坐标系下，为了得到最优轨迹，将优化目标也拆分成纵向和横向。横向上主要是车体在偏离车到中心线时能够在车辆的制动能力限制下相对安全和舒适地收敛到参考轨迹；纵向上主要考虑应对避障等因素引起的加速或减速时的舒适度，用Jerk加加速度来描述。因此可以将这两部分优化目标描述为：
$$ J[d,s] = J_d[d] + k_sJ_s[s] $$

$k_s$ 表示权重因子。将d(t)和s(t)统一用 $\xi_1(t)$ 表示，可以用微分方程的形式描述横向和纵向的运动学方程：
$$
\dot{\xi} =
  \begin{bmatrix}
   0 & 1 & 0 \\
   0 & 0 & 1 \\
   0 & 0 & 0
  \end{bmatrix} \xi +
  \begin{bmatrix}
  0 \\ 0 \\1
  \end{bmatrix} u = f(\xi,u)
$$
$\xi^T = [\xi_1, \xi_2, \xi_3]$ 表示相应的位置、速度和加速度，
 $u(t) = \dddot{\xi}(t)$ 则对应这jerk因子。定义损失函数：
$$
J_{\xi} := \int_{0}^{\tau}f_0(u)dt +(h(\xi(t),t))_{\tau} , f_0(u)=1/2u^2(t)
$$
对于无约束的 $\xi_1(t)$ ，使得从初始状态 $\xi_0$ 迁移到最终状态 $\xi_{\tau}$ 并且最小化损失函数 $J_{\xi}$ 的问题，已经在Takahashi的文章中证明这类Jerk最优化问题可以使用一个五次多项式表示。给定起始和终点的位置、速度和加速度约束，就可以根据这个结论求解出最优的 $\xi(t)$ ,也即相应的d(t)和s(t)。
注意到，起点和终点的配置不是固定的。通过一组在目标区域内的目标配置设定，包含 $\tau, s_{\tau}/d_{\tau}, \dot{s}_{\tau}/\dot{d}_{\tau}$ ，可以得到轨迹的备选集合，然后依据损失函数或者说最小化Jerk原则筛选出最优轨迹。
在实际实现中对采样生成轨迹做了一定的简化和改动，采用匀速模型将s(t)和d(t)的求解简化为d(s)的求解，只对横向进行优化，仅在 $\xi_{\tau}$ 附近做采样，后续可以逐步引入终点时间和横向偏移采样。筛选机制中将横向和纵向合并，利用横向的偏移、速度和加速度进行打分筛选。

![sample](resource/cost_function.png)

#### 算法输入
算法的输入主要包括：
* 通过当前目标车道中心线构造的frenet坐标系
* 本车当前frenet坐标
* 通过prediction构建的path time graph(具体可参考path_time_graph.cpp和后续对避障速度规划的介绍)
* 巡航速度、停止线距离等信息

#### 算法流程
在frenet坐标系下lattice算法首先根据本车当前的位置在参考路径
（目标车道中心线）在纵向和横向上分开采样。目前横向采样的纵向s采样点为{10, 20, 40, 80}，横向d采样点为{-0.5, 0, 0.5}（单位米）。由此确定路径形状后进行纵向采样，也即对终点速度进行采样，巡航场景下终点速度设置为巡航速度，加速度为0，时间采样点设置为{0.1, 1, 2, 3, 4, 5, 6, 7, 8}（单位秒）；同时针对停车场景增加对红灯停止线的纵向采样（终点速度为0，距离为外部输入的停止距离，加速度为0）。具体代码参考end_condition_sampler.cpp。

完成轨迹终点的采样后可以通过四次和五次多项式唯一求解出横向d(s)和纵向s(t)轨迹，并根据纵向加速度、巡航目标、横向偏移、横向加速度等指标对纵横向轨迹分别进行打分，然后对所有的纵横向轨迹组合方式进行排序，从最优轨迹开始进行constraint check和collision check，也即后筛选机制。最终得到满足动力学和无碰撞约束的轨迹，最终通过纵横向轨迹合并输出。代码具体参考latticeplanner.cpp.

当以上流程没有得到可行轨迹时会启动backup机制，生成一条加速度恒定的减速轨迹。下图是算法在变道过程的示意图。
![frenet frame in lane changing stage](resource/frenet_frame.png)

### 速度规划
实际上之前介绍的lattice算法已经同时求解了路径和速度规划问题，但是由于其每一帧都在做重规划，其生成轨迹的一致性和稳定性都需要进一步优化。
目前的框架中集成了comma基于mpc的速度规划算法，在循迹模式下建立纵向上跟车的优化问题，通过mpc求解出最优的速度剖面。

#### 1) 跟车速度规划
跟车速度规划目标比较明确：在有前车leader的情况下，实时计算出一个期望速度，使得本车能够完成跟车任务。在计算过程中主要考虑前车的相对位置和速度，以及本车的速度和加速度信息。
跟车速度规划采用基于MPC的优化求解方法，主要结构如下：
![visibility graph](resource/mpc_speed_planner.png)

#### 2) 避障速度规划
为了保证在本车沿着路径规划器给出的既定路线行驶时，对周围的动态障碍物进行合理的规避，速度规划需要在生成速度剖面的同时将对动态障碍物的预测轨迹考虑进去。目前采用的方法是纵向轨迹p固定，将动态障碍物转化到p-t二维空间，利用visibility graph 进行图优化求解。

目前在考虑障碍物时主要使用矩形近似（矩形和圆形的碰撞检测计算量小），在建立path-time space时将路径规划生成的Frenet轨迹转换到Cartesian坐标系下，从感知模块获取的关于前方动态障碍物的预测轨迹也同样转换到Cartesian坐标系下，对时间进行离散化，通过二维平面的碰撞检测，建立起p-t图：

![visibility graph](resource/path_time.png)

建立起p-t图之后，就将二维空间+一维时间的路径&速度规划避障问题转换到二维情况下，从而可以利用图论方法去计算出最优P(t)轨迹，得到速度剖面。一种方法是再将p_t图离散成gridmap，利用传统的A*等图优化方法计算出离散轨迹。但是这种方法计算复杂度高，并且不能得到连续的速度。目前采用的另一种基于可视图的方法。该方法基于的前提是将p-t中的障碍物区域近似成n矩形，在这种情况下，时间最优的轨迹一定由矩形的左上及右下顶点连接而成。在此基础上，可以将 $(p,t,v)$ 作为在p-t图上规划的状态量，按照时间次序对2n个顶点排序，利用迭代的方式不断更新每个顶点的可达速度区间。最后再计算出终点的可达速度以及时间区间。
![visibility graph](resource/vip.png)

在当前点的速度区间向目标点的速度区间更新时，根据速度限制和加速度限制得到在p-t图上的覆盖包络。规划中系统的运动学约束：
$$
\dot{p} = v \\
v \in [\underline{v}, \bar{v}] \\
\dot{v} \in [\underline{a}, \bar{a}]
$$

用 $P^+$ 和 $P^-$ 分别表示以最大加速度和最小加速度运动，$L$ 表示匀速运动。
主要考虑三种覆盖情况：通过先减速后加速 $P^-P^+$ 和先加速后减速 $P^+P^-$ 得到覆盖包线；达到速度上限需要增加 $P^-P^+L$ 和 $P^+P^--$ 两条包线；达到速度下限需要增加  $P^+P^-L$ 和 $P^-LP^+$ 两条包线。以第一种情况为例，以 $t_s$ 表示加速和减速的切换时间，可以得到包络方程：
$$
l(t,t_s) = (p_0, + tv_0 + 1/2(\underline{a}-\bar{a})t_s^2 + 1/2 \bar{a}t^2, v_0 + t_s(\underline{a}-\bar{a}) + \bar{a}t） \\
r(t,t_s) = (p_0, + tv_0 + 1/2(\bar{a} - \underline{a})t_s^2 + 1/2 \underline{a}t^2, v_0 + t_s(\bar{a} - \underline{a}) + \underline{a}t）
$$
结合 $p=p_f, t=t_f$ 的目标点位置条件，可以通过调整 $t_s$ 得到相应的目标点速度区间。
在得到覆盖区域的包线后需要根据p-t图中矩形障碍区域的顶点相对位置确定是否有碰撞，最终确定目标点的一个可行速度区间。
![visibility graph](resource/ppl.png)

具体算法参考:
http://motion.pratt.duke.edu/papers/ICRA2012-Johnson-optimal.pdf

##### 注意
以上描述的PVTP规划算法由于其时间最优性导致在实际仿真测试中并不适用，目前已经暂时搁置。
