# openrover_ros2

two support ros2 packages: lib-serial source code in libserial_vendor  and libVesc

ros2_control plugin

----------
docker setup instructions

docker build -t ros-dev .

Add for ps4 controller
--device=/dev/input/js0

docker run -v /home/countzero/repos/openrover_ros2:/ros_ws -td  --cap-add sys_ptrace -p127.0.0.1:2222:22 --name clion_remote_env ros-dev

sudo chmod a+rw /dev/input/js0

ssh user@localhost -p2222

bash
