rosbag for 105 control robot

#########如果roscd执行失败了，记得设置你当前终端下的ROS_PACKAGE_PATH环境变量，设置方法如下：
$ source ~/catkin_ws/devel/setup.bash 或者
$ export ROS_PACKAGE_PATH=~/indigo_workspace/sandbox:$ROS_PACKAGE_PATH
$ roscd beginner_tutorials
source ./devel/setup.bash

##########<----arduino,2018.3---->
arduino
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0

###########<----matlab---->
$ cd /usr/local/MATLAB/R2016b/bin
$ ./matlab # 如果是第一次运行，建议加sudo

##########<----change bashrc---->
gedit ~/.bashrc &
source ~/.bashrc

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws_iiwa/devel/setup.bash
source ~/ros_ws/devel/setup.bash
source ~/iiwa/devel/setup.bash

export ROS_IP=192.168.168.10
export ROS_MASTER_URI=http://$ROS_IP:11311

##########<---ros interface -->
catkin_make
rqt_plot rqt_plot

rosbag record -a
rosbag play <  >    rosbag play 2018-02-10-08-25-21.bag

rosbag play subset.bag /iiwa/state/CartesianPose:=/record_position

rosrun rosbag topic_renamer.py /iiwa/state/CartesianPose 2018-02-10-08-38-20.bag /record_position 2018-05-11-14-49-49.bag#这个命令不会使

##########<----Stanfod---->
cd catkin_ws_iiwa/devel/lib/iiwa/
./iiwa-bin


###########<------新建一个包-------->
cd <path_to_catkin_ws>
cd src
git clone ...
source /opt/ros/indigo/setup.bash

###########<------roslaunch运行新包里的程序------->
 cd <path_to_catkin_ws>
 source devel/setup.bash
　roslaunch ...

##########—————————ATI—————————————
 rosrun netft_control netft_node --address 192.168.1.1

##########————————————NDI——————————————
rosrun polaris_sensor polaris_sensor_node _roms:=/home/edward/8700339.rom _port:=/dev/ttyUSB0

##########——————————2018.6.24开关实验————————————
##二维
rosrun iiwa_ros pt_admittance_add_force_2d 
应该用
 rosrun iiwa_ros pt_zq1_2d
21开始是
##三维
rosrun iiwa_ros pt_admittance_add_force
应该用
 rosrun iiwa_ros pt_zq1
6开始是
##记录数据
 rosbag record -a -O wr_2018.6.24_2d_2emg_0.bag


#2emg实时
wr_2018.6.24_2d_2emg_1.bag
...
wr_2018.6.24_2d_2emg_6.bag
wr_2018.6.24_3d_2emg_1.bag
...
wr_2018.6.24_3d_2emg_6.bag
#6emg离线（2d和3d反了）
wr_2018.6.24_2d_6emg_1.bag
...
wr_2018.6.24_2d_6emg_6.bag
wr_2018.6.24_3d_6emg_1.bag
...
wr_2018.6.24_3d_6emg_6.bag

###########--------2018.6.26辨识实验----------------
###低中高三次
ndi:wrndi_bs_1  ATI:wrati_bs_1   emg:wremg_bs_1
###单项个三次
wrndi_bs_d_1
wrndi_bs_z_1
g
###方向各三次
wrndi_bs_x_1

###########--------ndi和git----------------
#NDI
#install
sh NDI_ToolBox_v5-001-017_for_Linux.sh
#run
cd opt/NDIToolBox/
sudo sh Track

#git
git init
git add .
git commit -m "change..."
###注意在pull和push时要注意是否版本有区别，对于长时间没搞过的，要对比一下在合并：https://blog.csdn.net/zcw4237256/article/details/78542122，对于每天都做的，那就
git pull
git push origin master
git push origin Runiiwa
