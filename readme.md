# IGVC AutoNav Simulator

## How to run

1. Running the sim
```bash
roslaunch simulation igvc_sim.launch
```
2. Running the 2d cost mapper
```bash
rosrun filter_pc fullnode.py
rosrun mapping2 octomaplauncher.launch
```
Visualize output on ``/projected_map`` map.
3. Running the goal calculator
```bash
rosrun goal_gen goalcalculator.py
```
Visualize output on ``/found_block`` marker or ``/move_base_simple/goal`` pose.
4. Running the motion planner
todo: add instructions to readme