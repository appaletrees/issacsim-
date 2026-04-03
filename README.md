# 基于 NVIDIA Isaac Sim（或等效仿真环境），实现一个简化的：机器人路径跟踪仿真 + 自动调参系统
# 注意所有程序运行后生成的数据保留文件都会在前一个文件夹

#完成一、仿真环境搭建
1.使用 Isaac Sim 创建一个基础场景（平地即可）
2.加载一个可运动的机器人模型（轮式机器人即可）
3.支持控制机器人运动（线速度 v + 角速度 ω）

#完成二、路径跟踪控制
1.实现一个PID路径跟踪算法
2.输入一个二维坐标，输出角速度、线速度
3.机器人沿着路径移动误差收敛

#完成三、数据采集
1.机器人轨迹
2.路径误差
3.控制器的输出

#完成四、算法评估
1.路径误差
2.控制器输出稳定性

#完成五、自动调参
通过pso算法自动整定PID参数


#---------------------------------------------------
#环境配置
issacsim 官网下载5.1版本，安装到C：\isaac-sim下

#小车仿真文件
car_4.usd 为小车仿真模型，支持v（角速度），w（线速度）差速驱动
![](https://github.com/appaletrees/issacsim-/blob/main/%E5%B0%8F%E8%BD%A6%E5%9B%BE%E7%89%87.png)

#使用方法
安装完issacsim后，进入C：\isaac-sim，新建一个文件夹命名为a_mytasks,然后把项目所有文件拉取到这个文件夹下

在C：\isaac-sim目录下.\python.bat .\a_mytasks\test2.py进行轨迹跟踪,并且保存跟踪结果到simulation_log.txt文件，通过运行analyze_results.py可以对他进行进一步分析
![](https://github.com/appaletrees/issacsim-/blob/main/%E8%BF%90%E8%A1%8C%E6%95%B0%E6%8D%AE%E7%9A%84%E5%88%86%E6%9E%90.png)
在C：\isaac-sim目录下.\python.bat .\a_mytasks\test3.py进行pso参数整定 运行结束后会生成最优PID参数，以及pso_convergence.png表示PID参数变化以及效果图

test4.py做了一个简单的轨迹规划，轨迹规划出速度而PID作为补偿
![]https://github.com/appaletrees/issacsim-/blob/main/test4.png
