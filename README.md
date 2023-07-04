This package will generate the collision objects to represent a human.  The collision objects can then be avoided when planning robot motions via moveit.plan() calls.  This package can operate in two modes: simulation or live.

In the simulation mode, the human pose sequence will be read from one of the csv files in the data folder of this package.  You run this in a terminal:
```
roslaunch human_collision_objects fake_human.launch human_task_num:=1
```
You can change the value of "human_task_num" to match the number of the pose sequence csv you wish to use. The launch files for the simulated human has a "transform" parameter used to properly align the recorded human with the workcell, in case you record in one cell and simulate in another cell.  It is an array in the format "x,y,z,q_w,q_x,q_y,q_z" where x,y,z is the point translation and q_w,q_x,q_y,q_z are quaternion elements for rotation. The current recordings in the data folder were recorded in a cell where x points to the left, y points at the human, z is up, and the origin is at table height.

In the live mode, the human poses is read for the ros topic /skeleton_points and then cylinders matching the live human pose are put into the planning scene.  Run this in a terminal:
```
roslaunch human_collision_objects live_human.launch
```
No transform is necessary for the live human.
