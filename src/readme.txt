sudo apt-get install libpcap-dev
需要进到rosdrive/cfg 
sudo chmod a+x FilterParams.cfg
运行为 roslaunch rfans_driver node_manager.launch

需要更改ip为192.168.0.xxx
需要更改 build_3D_map的订阅，由/velodyne_points 改为/rfans_driver/rfans_points
需要更改 tf_laser的转换，由velodyne改为world




# 运行
roslaunch rfans_driver node_manager.launch
roslaunch ugvc_navigation build_map.launch 

