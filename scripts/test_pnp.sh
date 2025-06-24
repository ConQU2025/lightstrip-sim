#!/bin/bash

# PnP算法测试脚本

echo "=== 紫色灯条PnP位姿估计测试 ==="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未设置"
    echo "请运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查catkin工作空间
if [ ! -f "/home/cromemadnd/catkin_ws/devel/setup.bash" ]; then
    echo "错误: catkin工作空间未编译"
    echo "请运行: cd /home/cromemadnd/catkin_ws && catkin_make"
    exit 1
fi

# 设置环境
source /home/cromemadnd/catkin_ws/devel/setup.bash

echo "1. 启动仿真环境..."
echo "   roslaunch conqu_lightstrip_sim strip.launch"
echo ""

echo "2. 在新终端中查看位姿输出:"
echo "   rostopic echo /lightstrip_pose"
echo ""

echo "3. 查看图像检测结果:"
echo "   观察弹出的OpenCV图像窗口"
echo ""

echo "4. 检查节点状态:"
echo "   rosnode list | grep lightstrip"
echo ""

echo "=== 参数说明 ==="
echo "- 相机参数在 config/lightstrip_params.yaml 中配置"
echo "- 灯条3D模型尺寸可调整 lightstrip_width 和 lightstrip_height"
echo "- HSV检测参数可根据光照条件调整"
echo ""

echo "=== 预期输出 ==="
echo "- 终端显示检测到的位姿信息"
echo "- 图像窗口显示绿色边界框、蓝色中心点和彩色坐标轴"
echo "- /lightstrip_pose 话题发布 geometry_msgs/PoseStamped 消息"

echo ""
echo "按 Ctrl+C 停止测试"
