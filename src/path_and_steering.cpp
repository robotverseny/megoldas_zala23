// publishes nav_msgs/Path and steering marker 

#include <iostream>
#include <string.h>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

const double wheelbase = 0.3187; // documention based | measured: ~32 cm
double steering_angle, speed_cmd; 
int path_size;
bool steering_enabled;
bool first_run = true, publish_steer_marker;
const double map_gyor_0_x = 697237.0, map_gyor_0_y = 5285644.0;
const double map_zala_0_x = 639770.0, map_zala_0_y = 5195040.0;
const float max = 1.5; // max speed TODO: what is the max speed of wheeltec
std::string marker_color;
ros::Publisher marker_pub, path_pub, text_pub;
nav_msgs::Path path;
geometry_msgs::Pose actual_pose;


float mapval(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Callback for /cmd_vel
void vehicleSteeringCallback(const geometry_msgs::Twist &cmd_msg){
    steering_angle = cmd_msg.angular.z;
    speed_cmd = cmd_msg.linear.x;

}

// Callback for pose messages
// rostopic echo /odom/pose/pose/position/x
void vehiclePoseCallback(const nav_msgs::Odometry &pos_msg){
    actual_pose = pos_msg.pose.pose;
    //ROS_INFO_STREAM(pos_msg.pose.pose);
}

void loop(){
    if (publish_steer_marker)
    {
        visualization_msgs::Marker steer_marker;
        steer_marker.header.frame_id = "base_link";
        steer_marker.header.stamp = ros::Time::now();
        steer_marker.ns = "steering_path";
        steer_marker.id = 0;
        steer_marker.type = steer_marker.LINE_STRIP;
        steer_marker.action = visualization_msgs::Marker::ADD;
        steer_marker.pose.position.x = 0;
        steer_marker.pose.position.y = 0;
        steer_marker.pose.position.z = 0;
        steer_marker.pose.orientation.x = 0.0;
        steer_marker.pose.orientation.y = 0.0;
        steer_marker.pose.orientation.z = 0.0;
        steer_marker.pose.orientation.w = 1.0;
        steer_marker.scale.x = 0.2;
        if(marker_color == "r"){
            steer_marker.color.r = 0.96f; steer_marker.color.g = 0.22f; steer_marker.color.b = 0.06f;
        }
        else if(marker_color == "b"){
            steer_marker.color.r = 0.02f; steer_marker.color.g = 0.50f; steer_marker.color.b = 0.70f;
        }
        else if(marker_color == "y"){ // yellow
            steer_marker.color.r = 0.94f; steer_marker.color.g = 0.83f; steer_marker.color.b = 0.07f;
        }
        else{ // gradient
            // green-blue-red gradient: https://coolors.co/gradient-maker/7fffbb-0088cc-e73666
            float green_r = 127./255., green_g = 255./255., green_b = 187./255.; // #7FFFBB 127, 255, 187
            float red_r = 231./255., red_g =54./255., red_b =102./255.; // #E73666 231, 54, 102
            float blue_r = 0./255., blue_g =136./255., blue_b =204./255.; // #0088CC 0, 136, 204
            if(speed_cmd < 0){
                steer_marker.color.r = green_r; steer_marker.color.g = green_g; steer_marker.color.b = green_b;
                ROS_INFO_STREAM("Negative speed (going backwards) " << speed_cmd << " m/s");
            }
            else if(speed_cmd > max){
                steer_marker.color.r = red_r; steer_marker.color.g = red_g; steer_marker.color.b = red_b;
                ROS_INFO_STREAM("Higher than " << max << "m/s (max) current ref: "<< speed_cmd << " m/s");
            }
            else if( 0 < speed_cmd && speed_cmd < max / 2){
                float c0 = mapval(speed_cmd, 0, max / 2, green_r, blue_r);
                float c1 = mapval(speed_cmd, 0, max / 2, green_g, blue_g);
                float c2 = mapval(speed_cmd, 0, max / 2, green_b, blue_b);
                steer_marker.color.r = c0; steer_marker.color.g = c1; steer_marker.color.b = c2;
            }
            else{
                float c0 = mapval(speed_cmd, max / 2, max, blue_r, red_r);
                float c1 = mapval(speed_cmd, max / 2, max, blue_g, red_g);
                float c2 = mapval(speed_cmd, max / 2, max, blue_b, red_b);   
                steer_marker.color.r = c0; steer_marker.color.g = c1; steer_marker.color.b = c2;
            }

        }
        steer_marker.color.a = 1.0;
        steer_marker.lifetime = ros::Duration();
        double marker_pos_x = 0.0, marker_pos_y = 0.0, theta = 0.0;
        if(speed_cmd < 0){ // negative speed
            for (int i = 0; i < 10; i++)
            {
                marker_pos_x += 0.01 * 10 * cos(theta);
                marker_pos_y += 0.01 * 10 * sin(theta);
                theta += 0.01 * 10 / wheelbase * tan(steering_angle);
                geometry_msgs::Point p;
                p.x = marker_pos_x;
                p.y = marker_pos_y;
                steer_marker.points.push_back(p);
            }            
            reverse(steer_marker.points.begin(), steer_marker.points.end());
            marker_pos_x = 0.0, marker_pos_y = 0.0, theta = 0.0;
            for (int i = 0; i < 10 + int(-20 * speed_cmd); i++)
            {
                marker_pos_x -= 0.01 * 10 * cos(theta);
                marker_pos_y -= 0.01 * 10 * sin(theta);
                theta -= 0.01 * 10 / wheelbase * tan(steering_angle);
                geometry_msgs::Point p;
                p.x = marker_pos_x;
                p.y = marker_pos_y;
                steer_marker.points.push_back(p);
            }
        }
        else{
            for (int i = 0; i < 10 + int(20 * speed_cmd); i++)
            {
                marker_pos_x += 0.01 * 10 * cos(theta);
                marker_pos_y += 0.01 * 10 * sin(theta);
                theta += 0.01 * 10 / wheelbase * tan(steering_angle);
                geometry_msgs::Point p;
                p.x = marker_pos_x;
                p.y = marker_pos_y;
                steer_marker.points.push_back(p);
            }
        }
        marker_pub.publish(steer_marker);
        steer_marker.points.clear();
    }
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position = actual_pose.position;
    pose.header.frame_id = "odom_combined";
    path.header.frame_id = "odom_combined";
    pose.pose.orientation = actual_pose.orientation; 
    path.poses.push_back(pose);
    path.header.stamp = ros::Time::now();
    path.poses.push_back(pose);
    // keep only the last n (path_size) path message
    if (path.poses.size() > path_size){
        int shift = path.poses.size() - path_size;
        path.poses.erase(path.poses.begin(), path.poses.begin() + shift);
    }
    path_pub.publish(path);

    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "base_link";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "text_marker";
    text_marker.id = 0;
    text_marker.type = text_marker.TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = 0.1;
    text_marker.pose.position.y = -1.2;
    text_marker.pose.position.z = 0.5;
    text_marker.pose.orientation.x = 0.0;
    text_marker.pose.orientation.y = 0.0;
    text_marker.pose.orientation.z = 0.0;
    text_marker.pose.orientation.w = 1.0;
    if(speed_cmd < 0){ // negative speed
        text_marker.color.r = 231./255.; // #E73666 231, 54, 102
        text_marker.color.g =  54./255.;
        text_marker.color.b = 102./255.;        
    }
    else{
        text_marker.color.r = 0.8;
        text_marker.color.g = 0.9;
        text_marker.color.b = 1.0;
    }
    text_marker.color.a = 1.0;
    text_marker.scale.z = 0.4;
    text_marker.lifetime = ros::Duration();
    char str[200];
    sprintf(str,"%.2fm/s", speed_cmd);
    text_marker.text = str;
    text_pub.publish(text_marker);

}

int main(int argc, char **argv)
{
    std::string pose_topic, marker_topic, path_topic;
    ros::init(argc, argv, "path_and_steering");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("pose_topic", pose_topic, "/odom");
    n_private.param<std::string>("marker_topic", marker_topic, "/marker_steering");
    n_private.param<std::string>("path_topic", path_topic, "/marker_path");
    n_private.param<std::string>("marker_color", marker_color, "gradient");
    n_private.param<bool>("publish_steer_marker", publish_steer_marker, true);
    n_private.param<int>("path_size", path_size, 100);
    ros::Subscriber sub_cmd = n.subscribe("/cmd_vel", 1, vehicleSteeringCallback);
    ros::Subscriber sub_current_pose = n.subscribe(pose_topic, 1, vehiclePoseCallback);
    marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic, 1);
    path_pub = n.advertise<nav_msgs::Path>(path_topic, 1);
    text_pub = n.advertise<visualization_msgs::Marker>("/marker_text", 1);
    ROS_INFO_STREAM("Node started: " << ros::this_node::getName() << " subscribed: " << pose_topic << " publishing: " << marker_topic << " " << path_topic);
    ros::Rate rate(20); // ROS Rate at 20Hz
    while (ros::ok()) {
        loop();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
