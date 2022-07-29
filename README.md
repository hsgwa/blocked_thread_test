# blocked_thread_test

## Install

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/

git clone https://github.com/hsgwa/blocked_thread_test.git src/blocked_thread_test
colcon build --symlink-install
```

## run

```
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # use fastdds.
ros2 run blocked_thread_test blocked_thread_test --ros-args -p thread_num:=10
```

## measure
```
top -p `pgrep blocked_thread_test`
```
