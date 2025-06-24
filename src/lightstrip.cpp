#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

cv::Scalar hsv_min, hsv_max;
// Global variables for contour filtering parameters
double area_min, area_max;
double area_ratio_min;
double aspect_ratio_min, aspect_ratio_max;

// Camera parameters
cv::Mat camera_matrix, dist_coeffs;
ros::Publisher pose_pub;

// 3D model of light strip (assuming rectangular light strip)
// 假设灯条是一个矩形，实际尺寸需要根据真实情况调整
vector<cv::Point3f> lightstrip_3d_points;

// 初始化灯条的3D模型点
void initLightstrip3DModel(double width, double height) {
    // 假设灯条是一个矩形，以灯条中心为原点
    // 4个角点的3D坐标（单位：米）
    lightstrip_3d_points.clear();
    lightstrip_3d_points.push_back(cv::Point3f(-width/2, -height/2, 0));  // 左下
    lightstrip_3d_points.push_back(cv::Point3f(width/2, -height/2, 0));   // 右下
    lightstrip_3d_points.push_back(cv::Point3f(width/2, height/2, 0));    // 右上
    lightstrip_3d_points.push_back(cv::Point3f(-width/2, height/2, 0));   // 左上
}

// 从轮廓获取有序的角点
vector<cv::Point2f> getOrderedCorners(const vector<cv::Point>& contour) {
    // 使用凸包算法获取轮廓的凸包
    vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    
    // 如果凸包点数少于4个，则无法进行PnP
    if (hull.size() < 4) {
        return vector<cv::Point2f>();
    }
    
    // 近似多边形以获得更规整的角点
    vector<cv::Point> approx;
    double epsilon = 0.02 * cv::arcLength(hull, true);
    cv::approxPolyDP(hull, approx, epsilon, true);
    
    // 如果近似后的点数少于4个，使用原始凸包
    if (approx.size() < 4) {
        approx = hull;
    }
    
    // 只取前4个点
    if (approx.size() > 4) {
        approx.resize(4);
    }
    
    // 将Point转换为Point2f
    vector<cv::Point2f> corners;
    for (const auto& pt : approx) {
        corners.push_back(cv::Point2f(pt.x, pt.y));
    }
    
    // 按照左下、右下、右上、左上的顺序排序
    if (corners.size() == 4) {
        // 计算中心点
        cv::Point2f center(0, 0);
        for (const auto& pt : corners) {
            center += pt;
        }
        center *= (1.0f / corners.size());
          // 按照角度排序
        vector<pair<double, cv::Point2f>> angle_points;
        for (const auto& pt : corners) {
            double angle = atan2(pt.y - center.y, pt.x - center.x);
            angle_points.push_back(make_pair(angle, pt));
        }
        sort(angle_points.begin(), angle_points.end(), [](const pair<double, cv::Point2f>& a, const pair<double, cv::Point2f>& b) {
            return a.first < b.first;
        });
        
        vector<cv::Point2f> ordered_corners;
        for (const auto& ap : angle_points) {
            ordered_corners.push_back(ap.second);
        }
        
        return ordered_corners;
    }
    
    return corners;
}

// 使用PnP算法计算位姿
bool solvePnP(const vector<cv::Point2f>& image_points, cv::Vec3d& rvec, cv::Vec3d& tvec) {
    if (image_points.size() < 4 || lightstrip_3d_points.size() < 4) {
        return false;
    }
    
    try {
        return cv::solvePnP(lightstrip_3d_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
    }
    catch (const cv::Exception& e) {
        ROS_ERROR("PnP solving failed: %s", e.what());
        return false;
    }
}

void imgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image_raw = cv_ptr->image;

    // Convert to HSV color space
    cv::Mat image_hsv;
    cvtColor(image_raw, image_hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    inRange(image_hsv, hsv_min, hsv_max, mask);

    // Find contours of light strips
    vector<vector<cv::Point>> lightContours;
    vector<cv::Vec4i> hierarchy;
    findContours(mask.clone(), lightContours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);    // Filter and detect rectangular light strips
    vector<cv::Point2f> lightstrip_centers;
    cv::Mat result_image = image_raw.clone();
    for (int i = 0; i < lightContours.size(); i++) {
        double area = cv::contourArea(lightContours[i]);
        // Filter by area using parameters
        if (area < area_min || area > area_max) continue;

        // Get bounding rectangle
        cv::Rect boundingRect = cv::boundingRect(lightContours[i]);
        
        // Check if it's rectangular (aspect ratio and area ratio)
        double rect_area = boundingRect.width * boundingRect.height;
        double area_ratio = area / rect_area;
        double aspect_ratio = (double)boundingRect.height / boundingRect.width;
        
        // Filter for rectangular light strips using parameters
        if (area_ratio < area_ratio_min) continue;
        if (aspect_ratio > aspect_ratio_max || aspect_ratio < aspect_ratio_min) continue;

        // Calculate center point
        cv::Point2f center;
        center.x = boundingRect.x + boundingRect.width / 2.0;
        center.y = boundingRect.y + boundingRect.height / 2.0;
        lightstrip_centers.push_back(center);
        
        // Draw bounding rectangle and center point
        rectangle(result_image, boundingRect, cv::Scalar(0, 255, 0), 2);
        circle(result_image, center, 5, cv::Scalar(255, 0, 0), -1);
        
        // PnP算法计算位姿
        vector<cv::Point2f> corner_points = getOrderedCorners(lightContours[i]);
        if (corner_points.size() >= 4) {
            cv::Vec3d rvec, tvec;
            if (solvePnP(corner_points, rvec, tvec)) {
                // 将旋转向量转换为旋转矩阵
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);
                
                // 发布位姿信息
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = "camera_frame";
                
                // 设置位置（相机坐标系）
                pose_msg.pose.position.x = tvec[0];
                pose_msg.pose.position.y = tvec[1];
                pose_msg.pose.position.z = tvec[2];
                
                // 将旋转矩阵转换为四元数
                tf2::Matrix3x3 tf_rotation(
                    rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                    rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                    rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
                );
                tf2::Quaternion tf_quaternion;
                tf_rotation.getRotation(tf_quaternion);
                
                pose_msg.pose.orientation = tf2::toMsg(tf_quaternion);
                
                // 发布位姿
                pose_pub.publish(pose_msg);
                
                // 在图像上显示位姿信息
                string pose_text = "Pos: (" + to_string(tvec[0]) + ", " + to_string(tvec[1]) + ", " + to_string(tvec[2]) + ")";
                cv::putText(result_image, pose_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
                
                // 绘制坐标轴
                vector<cv::Point3f> axis_points;
                axis_points.push_back(cv::Point3f(0, 0, 0));    // 原点
                axis_points.push_back(cv::Point3f(0.1, 0, 0));  // X轴
                axis_points.push_back(cv::Point3f(0, 0.1, 0));  // Y轴
                axis_points.push_back(cv::Point3f(0, 0, -0.1)); // Z轴
                
                vector<cv::Point2f> projected_axis;
                cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs, projected_axis);
                
                if (projected_axis.size() >= 4) {
                    cv::Point origin = projected_axis[0];
                    cv::line(result_image, origin, projected_axis[1], cv::Scalar(0, 0, 255), 3); // X轴-红色
                    cv::line(result_image, origin, projected_axis[2], cv::Scalar(0, 255, 0), 3); // Y轴-绿色
                    cv::line(result_image, origin, projected_axis[3], cv::Scalar(255, 0, 0), 3); // Z轴-蓝色
                }
                
                ROS_INFO("Lightstrip pose - Position: (%.3f, %.3f, %.3f), Distance: %.3f m", 
                         tvec[0], tvec[1], tvec[2], sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2]));
            }
            
            // 绘制检测到的角点
            for (size_t j = 0; j < corner_points.size(); j++) {
                circle(result_image, corner_points[j], 3, cv::Scalar(255, 255, 0), -1);
                cv::putText(result_image, to_string(j), corner_points[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
            }
        }
    }

    // 

    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lightstrip");
    ros::NodeHandle nh("~");

    ROS_INFO("Starting lightstrip node...");

    // 初始化相机参数
    double fx, fy, cx, cy;
    vector<double> dist_coeffs_vec;
    nh.param<double>("camera_fx", fx, 800.0);  // 焦距x
    nh.param<double>("camera_fy", fy, 800.0);  // 焦距y
    nh.param<double>("camera_cx", cx, 320.0);  // 主点x
    nh.param<double>("camera_cy", cy, 240.0);  // 主点y
    nh.param<vector<double>>("dist_coeffs", dist_coeffs_vec, vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    
    // 构建相机内参矩阵
    camera_matrix = (cv::Mat_<double>(3, 3) << 
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);
    
    // 构建畸变系数矩阵
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    for (size_t i = 0; i < min(dist_coeffs_vec.size(), (size_t)5); i++) {
        dist_coeffs.at<double>(i) = dist_coeffs_vec[i];
    }
    
    ROS_INFO("Camera parameters - fx: %.1f, fy: %.1f, cx: %.1f, cy: %.1f", fx, fy, cx, cy);
    
    // 初始化灯条3D模型
    double lightstrip_width, lightstrip_height;
    nh.param<double>("lightstrip_width", lightstrip_width, 0.2);   // 灯条宽度（米）
    nh.param<double>("lightstrip_height", lightstrip_height, 0.05); // 灯条高度（米）
    initLightstrip3DModel(lightstrip_width, lightstrip_height);
    
    ROS_INFO("Lightstrip 3D model - Width: %.3f m, Height: %.3f m", lightstrip_width, lightstrip_height);
    
    // 创建位姿发布器
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/lightstrip_pose", 1);

    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
    // 检测紫色的正确HSV范围
    nh.param<int>("h_min", h_min, 125);  // 紫色起始
    nh.param<int>("s_min", s_min, 50);   // 中等饱和度
    nh.param<int>("v_min", v_min, 50);   // 中等明度
    nh.param<int>("h_max", h_max, 155);  // 紫色结束
    nh.param<int>("s_max", s_max, 255);  // 最大饱和度
    nh.param<int>("v_max", v_max, 255);  // 最大明度
    hsv_min = cv::Scalar(h_min, s_min, v_min);
    hsv_max = cv::Scalar(h_max, s_max, v_max);

    // Read contour filtering parameters
    nh.param<double>("area_min", area_min, 100.0);      // 最小面积
    nh.param<double>("area_max", area_max, 50000.0);    // 最大面积
    nh.param<double>("area_ratio_min", area_ratio_min, 0.6);  // 面积填充比例
    nh.param<double>("aspect_ratio_min", aspect_ratio_min, 0.5);  // 最小长宽比
    nh.param<double>("aspect_ratio_max", aspect_ratio_max, 2.0);  // 最大长宽比

    ROS_INFO("HSV Thresholds - Min: (%d, %d, %d), Max: (%d, %d, %d)", 
             h_min, s_min, v_min, h_max, s_max, v_max);
    ROS_INFO("Contour Parameters - Area: %.1f-%.1f, Area ratio: %.2f, Aspect ratio: %.2f-%.2f",
             area_min, area_max, area_ratio_min, aspect_ratio_min, aspect_ratio_max);

    ros::Subscriber sub = nh.subscribe("/r2/cam_r2/image_raw", 1, imgCallback);

    ros::spin();

    cv::destroyAllWindows();
    return 0;
}
