rosbag for 105 control robot

#########如果roscd执行失败了，记得设置你当前终端下的ROS_PACKAGE_PATH环境变量，设置方法如下：
$ source ~/catkin_ws/devel/setup.bash 或者
$ export ROS_PACKAGE_PATH=~/indigo_workspace/sandbox:$ROS_PACKAGE_PATH
$ roscd beginner_tutorials
source ./devel/setup.bash

###查询历史命令
Ctrl+r

##########    <----arduino,2018.3---->
arduino
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0

###########    <----matlab---->
$ cd /usr/local/MATLAB/R2016b/bin
$ ./matlab # 如果是第一次运行，建议加sudo

##########    <----change bashrc---->
gedit ~/.bashrc &
source ~/.bashrc

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws_iiwa/devel/setup.bash
source ~/ros_ws/devel/setup.bash
source ~/iiwa/devel/setup.bash

export ROS_IP=192.168.168.10
export ROS_MASTER_URI=http://$ROS_IP:11311

##########    <---ros interface -->
catkin_make
rqt_plot rqt_plot

rosbag record -a
rosbag play <  >    rosbag play 2018-02-10-08-25-21.bag

rosbag play subset.bag /iiwa/state/CartesianPose:=/record_position

rosrun rosbag topic_renamer.py /iiwa/state/CartesianPose 2018-02-10-08-38-20.bag /record_position 2018-05-11-14-49-49.bag#这个命令不会使

##########   <----Stanfod---->
cd catkin_ws_iiwa/devel/lib/iiwa/
./iiwa-bin


###########  <------新建一个git包-------->
cd <path_to_catkin_ws>
cd src
git clone ...
source /opt/ros/indigo/setup.bash

######### ----------新建ros包-------------
cd src
catkin_create_pkg 包的名 actionlib message_generation roscpp rospy std_msgs actionlib_msgs
cd ..
catkin_make
source devel/setup.bash	

###########<------roslaunch运行新包里的程序------->
 cd <path_to_catkin_ws>
 source devel/setup.bash
　roslaunch ...

########## ——————————2018.6.24开关实验————————————
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

###########  --------2018.6.26辨识实验----------------
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

#########--------MYO----------------
 rosrun ros_myo myo-rawNode.py    打开myo通讯
 rosrun ros_myo emg_ascii_graph.py   获取肌电信号


########----------------git---------------------------
###  创建一个本地仓并上传
git init  #初始化
git statu   #查看当前状态
git add .     #增加文档
git commit -m "change..."     #设置备注
git remote add origin git@github.com:RLoad/xxxx.git
git push -u origin master

###  本地与远程的同步：https://blog.csdn.net/zcw4237256/article/details/78542122

###  注意只同步SRC文件夹中内容用上面命令，整个文件夹就直接push到master
###  标准做法
git status // 查看版本库的状态
git add .|[file you want add like README.md] // 添加修改的文件进入版本库
git commit -m "the content of your modify" // 提交版本库
git push -u origin master // 上传到远程版本库

###  注意在pull和push时要注意是否版本有区别
git pull   #只有在需要下载的时候才运行
git push -u origin master  #第一个最常用 -u代表



#######-----------gazebo 仿真----------------------------
首先记住要讲gazebo的launch文件中的控制器类型改成关节控制
 roslaunch iiwa_gazebo iiwa_gazebo.launch 打开仿真

  rosrun rqt_gui rqt_gui  设置初始位置 每个关节：
/iiwa/PositionJointInterface_J1_controller/command
配置文件保存在iiwa_gazebo/launch文件夹下

  rqt_plot rqt_plot   画刚度曲线
rostopic echo /iiwa/joint_states 查看关节数据是否准确
rosrun iiwa_ros joint_position_for_gazebo 运行正逆运动学并发送关节指令
rosrun iiwa_ros pt_sj_go_SEDS_for_gazebo 运行SEDS

######------------SEDS_GMM调试    最后rqt不能显示pose_go，所以用话题报记录之后用MATLAB处理,并git保存在kuka_ds_gm和5_wr_SEDS_GMM中
运行程序：rosrun iiwa_ros wr_SEDS_GMM.cpp
打开画图程序 rqt
画位置和刚度曲线：/iiwa/state/CartesianPose/pose/position/xyz和/pose_go/pose/position/xyz进行对比分析
配置保存在iiwa_ros/src/SEDS_GMM中

记录话题包：
mkdir ~/bagfiles/wr_SEDS_GMM
cd ~/bagfiles/wr_SEDS_GMM
1.记录纯位置控制下的位置数据（实际位置和命令位置）
rosbag record -O wr_seds_GMM_position_1.bag /iiwa/command/CartesianPose /iiwa/state/CartesianPose /pose_go
2.记录变阻抗控制下的位置数据（实际位置和命令位置）
rosbag record -O wr_seds_GMM_impedance_1.bag /iiwa/command/CartesianPose /iiwa/state/CartesianPose /pose_go

matlab处理
用了新命令timeseries来处理，十分方便
为了下次节省时间，可以保存整个工作空间：file --> sace workspace as




######-----------传感器同步---------------------
网卡地址分配，主网卡connect-yumi，192.168.168.10 255.255.255.0
副网卡ATI 192.168.1.68 255.255.255.0 gate192.168.1.1 DNS192.168.1.10
##########—————————ATI—————————————
 rosrun netft_control netft_node --address 192.168.1.1

##########————————————NDI——————————————
rosrun polaris_sensor polaris_sensor_node _roms:=/home/edward/8700339.rom _port:=/dev/ttyUSB0

#######之后打开机器人，在运行
rosrun iiwa_ros get_all_sensor_in_one_ApproximateTime




######-----------get all sensor in all 调试-----------------
1记录数据
rosbag record -O wr_sensor_together_test_1.bag /netft_data /polaris_sensor/targets

2,运行程序,回放话题，运行命令之后再次记录 
rosrun iiwa_ros get_all_sensor_in_one_ApproximateTime

rosbag record -O wr_sensor_together_look_1.bag /netft_data /polaris_sensor/targets /sensor_together


#######-----------用terminator自动打开多个窗口-----------------
1.保存当前窗口分布，并给每个窗口输入初始命令，加 ；bash
2.退出，关闭，新开窗口运行 terminator -l layoutname
具体terminator的配置可以直接用src文件中的terminator文件夹下config


######------------数据同步后的新阻抗辨识实验---------
0.进行NDI标定：上传
	1.虚拟机链接NDI，打开ros，标定程序中的MATLAB文件要放到MATLAB工作目录下
	2.运行 cd src/ndi_calibrate/ 在运行 python calibrate.py 得到TJM.txt,就是旋转矩阵，并复制到MATLAB 6_..程序中。
1.先关闭ndi，拔下USB，之后要先连好ati，ATI要在网页中归零，不要先运行ati命令，连好之后再连ndi，先运行NDI命令，在运行ati，不然target或者ATI不能读数。
2.运行程序，修改其中txt名字编号
rosrun iiwa_ros get_all_sensor_in_one_ApproximateTime   如果要分开的数据就运行rosrun iiwa_ros get_all_sensor_in_one_separite_txt
2.mathlab程序处理数据，计算末端刚度
   6_sensor_synchronizer中
3.阻抗向机器人传递的程序
