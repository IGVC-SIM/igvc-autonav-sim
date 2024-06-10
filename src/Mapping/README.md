## To run rtabmapping on your environment, download this file to your workspace and simply run:
### roslaunch rtab_mapping.launch hector:=true camera:=false

## If you download the package to a package(say mapping) in your workspace, run:
### cd catkin_ws/
### catkin_make
### roslaunch mapping rtab_mapping.launch hector:=true camera:=false


#### Make sure the scan, baseframe and odom topic in the launch file are set to the same topics that your bot is publishing onto.
