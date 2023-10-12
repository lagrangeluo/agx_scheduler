# !/bin/bash

docker start agx_scheduler &&
docker exec -it agx_scheduler /bin/bash /home/schedule_ws/src/agx_scheduler/start_launch.bash
