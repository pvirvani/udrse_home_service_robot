/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

double p_pos[2]  = {1, -8};
double d_pos[2] = {10, -12};

double pose[2] = {0, 0};  // current pose
bool pickupreached;
bool dropoffreached;
void current_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose[0] = msg->pose.pose.position.x;
  pose[1] = msg->pose.pose.position.y;
}

bool pickup_reached(double g_pos[2]){

    // ROS_INFO("Compute distance function: \n g0: %f \t g1: %f \n c0: %f \t c1: %f", g_pos[0], g_pos[1], c_pos[0],c_pos[1]);

    double dx = g_pos[0] - pose[0];
    double dy = g_pos[1] - pose[1];
    //computing total distance
    double t_dis = abs(sqrt(pow(dx,2) + pow(dy,2)));
    
    ROS_INFO("Pick-Up Distance t_dis is : %f", t_dis);
    
    if(t_dis < 2.5){
        pickupreached = true;
    }
    return (pickupreached);
}

bool dropoff_reached(double g_pos[2]){

    // ROS_INFO("Compute distance function: \n g0: %f \t g1: %f \n c0: %f \t c1: %f", g_pos[0], g_pos[1], pose[0],pose[1]);

    double dx = g_pos[0] - pose[0];
    double dy = g_pos[1] - pose[1];
    //computing total distance
    double t_dis = abs(sqrt(pow(dx,2) + pow(dy,2)));

    ROS_INFO("Drop-off Distance t_dis is : %f", t_dis);

    if(t_dis < 2.3){
       dropoffreached = true;
    }
    return (dropoffreached);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  

  ros::Subscriber pose_sub = n.subscribe("odom", 10, current_pose);
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  ROS_INFO("Going to pick up the virtual object from pick up point ... ");

  int state = 1;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();


    marker.ns = "add_markers";
    // marker.ns = "add_markers";
    marker.id = 0;
    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = 0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

        

    ros::spinOnce();

    if (state == 1) {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = p_pos[0];
      marker.pose.position.y = p_pos[1];
      marker_pub.publish(marker);
      pickupreached = pickup_reached(p_pos);
      ROS_INFO("Pick-Up Reached: %d", pickupreached);
      if (pickupreached) {
        sleep(5);
        ROS_INFO("Going towards the drop area ... ");
        state = 2;
      }
    }
    else if (state == 2) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker.pose.position.x = d_pos[0];
      marker.pose.position.y = d_pos[1];
      marker_pub.publish(marker);
      dropoffreached = dropoff_reached(d_pos);
      if (dropoffreached) {
        ROS_INFO("Reached at the drop area. ");
        state = 3;
      }
    }
    else /* state == 3 */ {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = d_pos[0];
      marker.pose.position.y = d_pos[1];
      marker_pub.publish(marker);
    }
  }

}


