# 更新详情
新增了适配ego的几个函数，将原有的避障换为ego
对main进行了一定的修改，先验证ego可跑
新增了仿真的shell

# 更新适配
把ego_ws/src下的五个文件夹整体迁移到first_task_ws/src下，和flight_mission同级，此时src下加上flight和livox应有7个文件夹

# 部分问题
参数还有待改进，该版本只是可跑，据观察有时贴障碍物较近，路径过于弯曲
降落时会反弹

# 待办
可以考虑加逻辑，在ego快到目标点时切定点，减少一点延迟时间
实机shell暂时没写
实机如果要用yolo需要找到库的路径并且修改cmake
模型检查中onFrame，还存在一定逻辑未补充，无人机投放函数暂未添加，如需添加请加至flight_control
ego的template里有两个绕圈函数，如果要修改可参考template.h
某些参数可从yaml读，暂时还未添加

# 注意！！！
如果要用ego一定要搭配main内的延迟使用
可以直接启动新shell，如果想要看到更详细的信息可以用下列的命令
# 终端1: 启动 roscore 和 Gazebo
roscore &
sleep 2
roslaunch tutorial_gazebo sim.launch

# 终端2: 启动 laser_to_worldframe
source ~/first_task_ws/devel/setup.bash
rosrun mission laser_to_worldframe

# 终端3: 启动 EGO 规划器
source ~/first_task_ws/devel/setup.bash
roslaunch mission ego_planner_mid360.launch

# 终端4: 检查话题
source ~/first_task_ws/devel/setup.bash
rostopic list | grep position_cmd
rostopic hz /position_cmd

# 终端5: 运行你的程序
source ~/first_task_ws/devel/setup.bash
roslaunch flight_mission flight_mission.launch