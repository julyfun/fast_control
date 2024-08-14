function jst {
    colcon build --symlink-install && source ./install/setup.bash && ros2 run $1 $1_node
}
function j.b {
    colcon build --symlink-install
}
function j.s {
    source install/setup.bash
}
function j.r {
    ros2 run $1 $1_node
}
function j.p0 {
    ros2 topic pub -1 /cali/p0 std_msgs/msg/Int32 "{data: $1}" 
}
function j.px {
    ros2 topic pub -1 /cali/px std_msgs/msg/Int32 "{data: $1}" 
}
function j.py {
    ros2 topic pub -1 /cali/py std_msgs/msg/Int32 "{data: $1}" 
}
function j.cali {
    ros2 topic pub -1 /cali/cali_fixed std_msgs/msg/Empty "{}"
}
function j.up {
    ros2 topic pub -1 /cali/upright std_msgs/msg/Int32 "{data: $1}" 
}
function j.s {
    ros2 topic pub -1 /cali/save std_msgs/msg/Int32 "{data: $1}" 
}
function j.l {
    ros2 topic pub -1 /cali/load std_msgs/msg/Int32 "{data: $1}" 
}
# print canbus sub
function j.pct {
    ros2 topic pub -1 /ik/pct std_msgs/msg/Empty "{}"
}

