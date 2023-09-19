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

## DESIGN Manual
1. 关于add_waypoint服务和add_lane服务:
waypoint的优先级更高，如果导航yaml文件为空，或者楼层缺失等等，调用add_waypoint可以完成创建文件，创建楼层等工作，但是add_lane服务没有这些功能，它的职责更加单一：只能在所有元素齐全的情况下才能创建一条单向lane，否则失败

## service
- add_waypoint
    - name: /agx_scheduler_node/add_waypoint_srv
    - type: agx_scheduler::add_waypoint
    - request:
    - response:
- add_lane
    - name: /agx_scheduler_node/add_lane_srv
    - type: agx_scheduler::add_lane
    - request:
    - response:
- delete_lane
    - name: /agx_scheduler_node/delete_lane_srv
    - type: agx_scheduler::delete_lane
    - request:
    - response: