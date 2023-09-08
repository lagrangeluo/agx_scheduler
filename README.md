# agx_scheduler respository
## TODO
1. 添加waypoint的时候使用最近邻搜索，判断自己的位置是否离最近的某个点太近，太近创建失败并且返回error_code和info
2. 支持顺序撤销操作，支持单独删除某个点，并删除和点相连的所有lanes
3. 支持撤销后的前进操作
4. 地图上的lanes可视化的粗细度，应该和车的直径相当
5. add_waypoint返回值包含路径点index，好像也可以不用？