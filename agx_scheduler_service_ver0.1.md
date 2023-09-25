# agx_scheduler service
## DESIGN Manual
1. 关于add_waypoint服务和add_lane服务:
waypoint的优先级更高，如果导航yaml文件为空，或者楼层缺失等等，调用add_waypoint可以完成创建文件，创建楼层等工作，但是add_lane服务没有这些功能，它的职责更加单一：只能在所有元素齐全的情况下才能创建一条单向或者双向lane，否则失败

## service
- add_waypoint（添加路径点）
    - name: /agx_scheduler_node/add_waypoint_srv
    
    - type: agx_scheduler::add_waypoint
    
    - request&response
    
      ```
      #the nav file name
      string file_name
      
      #specify the floor name,a nav map may contain multy floors
      string floor_name
      
      #the location of waypoint
      float64 location_x
      float64 location_y
      
      #the name of waypoint(optional)
      string waypoint_name
      ---
      #the bool flag to judge if set success
      bool success
      
      #the index of the waypoint
      uint8 index
      ```
    
      
    
- add_lane（添加路径）
    - name: /agx_scheduler_node/add_lane_srv
    
    - type: agx_scheduler::add_lane
    
    - request&response
    
      ```
      #the nav file name
      string file_name
      
      #specify the floor name,a nav map may contain multy floors
      string floor_name
      
      #the waypoint index of lane
      uint16 start_index
      uint16 end_index
      
      #if the lane is double-side
      bool double_side
      ---
      #the bool flag to judge if set success
      bool success
      uint8 index
      ```
    
      
    
- delete_lane（删除路径）
    - name: /agx_scheduler_node/delete_lane_srv
    
    - type: agx_scheduler::delete_lane
    
    - request&response
    
      ```
      #the nav file name
      string file_name
      
      #specify the floor name,a nav map may contain multy floors
      string floor_name
      
      #the waypoint index of lane
      uint16 start_index
      uint16 end_index
      
      #if the lane is double-side
      bool double_side
      ---
      #the bool flag to judge if set success
      bool success
      uint8 index
      ```
    
      
    
- delete_waypoint（删除路径点）
    - name: /agx_scheduler_node/delete_waypoint
    
    - type: agx_scheduler::delete_waypoint
    
    - request&response
    
      ```
      #the nav file name
      string file_name
      
      #specify the floor name,a nav map may contain multy floors
      string floor_name
      
      #the index of waypoint
      uint8 index
      ---
      #the bool flag to judge if set success
      bool success
      ```
    
      
    
- comfirm_update（确认导航图已经更新，重新调用构造函数）
    - name: /agx_scheduler_node/comfirm_update
    
    - type: agx_scheduler_node::comfirm_update
    
    - request&response
    
      ```
      #if we want to reconstruct the nav graph node which is running using the new yaml file,we should call this service
      #the new file name that we want to load
      string new_file_name
      ---
      bool success
      ```
    
      

