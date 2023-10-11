# !/bin/bash

docker start agx_scheduler &&
docker exec -it agx_scheduler /home/schedule_ws/src/agx_scheduler/start_launch.bash
