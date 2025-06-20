#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>

using namespace cv;
using namespace std;

Scalar hsv_min, hsv_max;
// Global variables for contour filtering parameters
double area_min, area_max;
double area_ratio_min;
double aspect_ratio_min, aspect_ratio_max;

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

    Mat image_raw = cv_ptr->image;

    // Convert to HSV color space
    Mat image_hsv;
    cvtColor(image_raw, image_hsv, COLOR_BGR2HSV);    // Mark the purple color range
    Mat mask;
    inRange(image_hsv, hsv_min, hsv_max, mask);

    // Find contours of light strips
    vector<vector<Point>> lightContours;
    vector<Vec4i> hierarchy;
    findContours(mask.clone(), lightContours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Filter and detect rectangular light strips
    vector<Point2f> lightstrip_centers;
    Mat result_image = image_raw.clone();
      for(int i = 0; i < lightContours.size(); i++) {
        double area = contourArea(lightContours[i]);
        
        // Filter by area using parameters
        if(area > area_min && area < area_max) {
            // Get bounding rectangle
            Rect boundingRect = cv::boundingRect(lightContours[i]);
            
            // Check if it's rectangular (aspect ratio and area ratio)
            double rect_area = boundingRect.width * boundingRect.height;
            double area_ratio = area / rect_area;
            double aspect_ratio = (double)boundingRect.width / boundingRect.height;
            
            // Filter for rectangular light strips using parameters
            if(area_ratio > area_ratio_min && 
               (aspect_ratio > aspect_ratio_max || aspect_ratio < aspect_ratio_min)) {
                // Calculate center point
                Point2f center;
                center.x = boundingRect.x + boundingRect.width / 2.0;
                center.y = boundingRect.y + boundingRect.height / 2.0;
                lightstrip_centers.push_back(center);
                
                // Draw bounding rectangle and center point
                rectangle(result_image, boundingRect, Scalar(0, 255, 0), 2);
                circle(result_image, center, 5, Scalar(255, 0, 0), -1);
                
                // Display center coordinates
                // char text[100];
                // sprintf(text, "(%.1f, %.1f)", center.x, center.y);
                // putText(result_image, text, Point(center.x + 10, center.y - 10), 
                //        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
                
                // ROS_INFO("Light strip %d center: (%.2f, %.2f)", 
                //         lightstrip_centers.size(), center.x, center.y);
            }
        }
    }
    
    // Display total number of detected light strips
    // ROS_INFO("Total light strips detected: %d", (int)lightstrip_centers.size());

    imshow("Detection Result", result_image);
    waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lightstrip");
    ros::NodeHandle nh("~");

    ROS_INFO("Starting lightstrip node...");    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
    // 检测紫色的正确HSV范围
    nh.param<int>("h_min", h_min, 125);  // 紫色起始
    nh.param<int>("s_min", s_min, 50);   // 中等饱和度
    nh.param<int>("v_min", v_min, 50);   // 中等明度
    nh.param<int>("h_max", h_max, 155);  // 紫色结束
    nh.param<int>("s_max", s_max, 255);  // 最大饱和度
    nh.param<int>("v_max", v_max, 255);  // 最大明度
    hsv_min = Scalar(h_min, s_min, v_min);
    hsv_max = Scalar(h_max, s_max, v_max);

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
