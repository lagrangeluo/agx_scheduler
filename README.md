# agx_scheduler respository
## TODO
- [ ] 添加waypoint的时候使用最近邻搜索，判断自己的位置是否离最近的某个点太近，太近创建失败并且返回error_code和info
- [ ] 支持顺序撤销操作，支持单独删除某个点，并删除和点相连的所有lanes
- [ ] 支持撤销后的前进操作
- [ ] 地图上的lanes可视化的粗细度，应该和车的直径相当
- [x] add_waypoint返回值包含路径点index，好像也可以不用？
- [ ] 充电：两个服务，一个是直接搜索找到最近的充点电，可能得用djistra来搜索最近的，一个是复用goal目标直接前往充电点充电
- [ ] lane和waypoint的多种服务
- [ ] 添加lane操作的耗时有点高，最高可以干到将近20ms，想办法优化代码流程
- [ ] 搞一个导航图检索模块，导航图每添加一段路径就执行检测程序，如果出现异常情况，比如离障碍物太近，反回false，ui上路径变为红色；若添加成功返回true，ui显示为绿色
- [ ] 支持添加弯曲路网，增加计算弯曲路线长度的功能
- [x] 对非法request的异常判断，并反馈
- [ ] 支持路网规划导航图的热修改，但必须是规划之前才能修改，正在规划中的话需要获取信号量或者获取锁才能操作
- [ ] 删除waypoint：暂时采用删除所有相关lane的方案，实际还会占用yaml文件位置以及graph的位置，长期使用会有问题
- [ ] 明确shared_ptr如果赋值另外一个，之前管理的内存会不会自动析构，确认是否需要手动析构一些指针
- [ ] update nav graph的时候直接重新申请了方法指针，可以调用更新函数这样不用重新构建类
- [ ] 增加栅格地图的坐标转化功能，方便以后接入rmf的坐标系
- [ ] service添加错误反馈，如果调用发生错误，将错误string通过response传递
- [ ] 与其他代码的耦合问题：将colcon build工具链的依赖改成支持catkin_make的，或者能够直接安装到install目录提供使用
- [ ] rmf_traffic仓库在melodic下编译失败，原因未知，暂时可以更改std::optional的返回值，不初始化rmf_traffice类来解使用
- [x] bash脚本方便启动容器和启动程序
- [ ] 反馈的路径应该包含车体的方向，现阶段没有方向，后期加上
- [x] 每次调用完搜索方法，重新初始化

## Support ros version
- noetic

## Build Respository
### create workspace
```shell
$ mkdir scheduler_ws && cd scheduler_ws
$ mkdir src && cd src
$ git clone https://github.com/lagrangeluo/agx_scheduler.git
```
### install colcon build
```shell
$ sudo apt install python3-colcon-common-extensions
```
## Start Node

```shell
$ roslaunch agx_scheduler agx_scheduler.launch
```
if use docker,run shell script:
```shell
$ sudo bash docker_start.bash
```
## DESIGN Manual
1. 关于add_waypoint服务和add_lane服务:
waypoint的优先级更高，如果导航yaml文件为空，或者楼层缺失等等，调用add_waypoint可以完成创建文件，创建楼层等工作，但是add_lane服务没有这些功能，它的职责更加单一：只能在所有元素齐全的情况下才能创建一条单向lane，否则失败

## service
- add_waypoint（添加路径点）
    - name: /agx_scheduler_node/add_waypoint_srv
    - type: agx_scheduler::add_waypoint
    - request:
    - response:
- add_lane（添加路径）
    - name: /agx_scheduler_node/add_lane_srv
    - type: agx_scheduler::add_lane
    - request:
    - response:
- delete_lane（删除路径）
    - name: /agx_scheduler_node/delete_lane_srv
    - type: agx_scheduler::delete_lane
    - request:
    - response:
- delete_waypoint（删除路径点）
    - name: /agx_scheduler_node/delete_waypoint
    - type: agx_scheduler::delete_waypoint
    - request:
    - response:
- comfirm_update（确认导航图已经更新，重新调用构造函数）
    - name: /agx_scheduler_node/comfirm_update
    - type: agx_scheduler_node::comfirm_update
    - request:
    - response: