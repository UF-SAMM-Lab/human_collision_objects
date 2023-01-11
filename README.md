This package will generate the collision objects to represent a human.  The collision objects can then be avoided when planning robot motions via moveit.plan() calls.  This package can operate in two modes: simulation or live.

In the simulation mode, the human pose sequence will be read from one of the csv files in the data folder of this package.  You run this in a terminal:
```
roslaunch human_collision_objects fake_human.launch human_task_num:=1
```
You can change the value of "human_task_num" to match the number of the pose sequence csv you wish to use.
