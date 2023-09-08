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

## DESIGN Manual
1. 关于add_waypoint服务和add_lane服务:
waypoint的优先级更高，如果导航yaml文件为空，或者楼层缺失等等，调用add_waypoint可以完成创建文件，创建楼层等工作，但是add_lane服务没有这些功能，它的职责更加单一：只能在所有元素齐全的情况下才能创建一条单向lane，否则失败