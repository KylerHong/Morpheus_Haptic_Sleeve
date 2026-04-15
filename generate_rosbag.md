IN VS CODE

to run robotic arm code you need to be in: 
`root@BEAR:~/catkin_ws`

then you run:
`roslaunch morpheus_hera cpne.launch`

create Session folder in morpheus_data/data folder e.g. NSNH01_Session2 
add all files generated during testing

to generate rosbag, navigate to:
`catkin_ws/src/morpheus_data/scripts#`

so from root@BEAR:~/catkin_ws:
`cd src`
`cd morpheus_data`
`cd scripts`

the final directory you should be sitting in is: 
`root@BEAR:~/catkin_ws/src/morpheus_data/scripts#`

now run: 
`python3 readBag.py ~/catkin_ws/src/morpheus_data/data/[YYYYMMDD]_[SUBJECT]_SessionX/*.bag`

rosbag should be generated

IN TERMINAL

to unlock the files, in terminal (!! - navigate to bear@BEAR): 
`sudo chown -R $USER:$USER ~morpheus_git/Morpheus/morpheus_data/data/[YYYYMMDD]_[SUBJECT]_Session[X]`
