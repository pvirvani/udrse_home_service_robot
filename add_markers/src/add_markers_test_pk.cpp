#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

//   while (ros::ok())
//   {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // ---------- Adding Marker at Pick-Up Zone ----------------- 

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 1;
    marker.pose.position.y = -5;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    ROS_INFO("pick-up marker added");

    // // Cycle between different shapes
    // switch (shape)
    // {
    // case visualization_msgs::Marker::CUBE:
    //   shape = visualization_msgs::Marker::SPHERE;
    //   break;
    // case visualization_msgs::Marker::SPHERE:
    //   shape = visualization_msgs::Marker::ARROW;
    //   break;
    // case visualization_msgs::Marker::ARROW:
    //   shape = visualization_msgs::Marker::CYLINDER;
    //   break;
    // case visualization_msgs::Marker::CYLINDER:
    //   shape = visualization_msgs::Marker::CUBE;
    //   break;
    // }

    // r.sleep();
//   }

// ----------- Adding Marker at Drop-Off Position ------------

ros::Duration(10.0).sleep();
ROS_INFO("pick-up marker removed and wait");

visualization_msgs::Marker drop_marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    drop_marker.header.frame_id = "map";
    drop_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    drop_marker.ns = "add_markers";
    drop_marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    drop_marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    drop_marker.action = visualization_msgs::Marker::ADD;

    // ---------- Adding Marker at Pick-Up Zone ----------------- 

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    drop_marker.pose.position.x = 10;
    drop_marker.pose.position.y = -12;
    drop_marker.pose.position.z = 0;
    drop_marker.pose.orientation.x = 0.0;
    drop_marker.pose.orientation.y = 0.0;
    drop_marker.pose.orientation.z = 0.0;
    drop_marker.pose.orientation.w = 0.3;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    drop_marker.scale.x = 0.5;
    drop_marker.scale.y = 0.5;
    drop_marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    drop_marker.color.r = 0.0f;
    drop_marker.color.g = 1.0f;
    drop_marker.color.b = 0.0f;
    drop_marker.color.a = 1.0;

    drop_marker.lifetime = ros::Duration();
    // ROS_INFO("d-up marker added")

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the drop_marker");
      sleep(1);
    }
    marker_pub.publish(drop_marker);
    ROS_INFO("drop-off marker added");

    ros::Duration().sleep();

return 0;


}