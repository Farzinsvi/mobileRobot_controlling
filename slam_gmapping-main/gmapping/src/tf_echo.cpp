 #include <cstdio>
 #include "tf/transform_listener.h"
 #include "ros/ros.h"
 #include <nav_msgs/Odometry.h>
 #include <geometry_msgs/Quaternion.h>
 #include <geometry_msgs/Twist.h>
 
 #define _USE_MATH_DEFINES
    

 double x,y,yaw_;
 
 
 class echoListener
 {
 public:
 
   tf::TransformListener tf;
 
   //constructor with name
   echoListener()
   {
 
   }
 
   ~echoListener()
   {
 
   }
 
 private:
 
 };
 
 void odomCallback(const nav_msgs::Odometry::ConstPtr msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
}
 
 
 int main(int argc, char ** argv)
 {
   //Initialize ROS
   ros::init(argc, argv, "tf_echo", ros::init_options::AnonymousName);
 
   ros::NodeHandle nh("~");
 
   ros::Subscriber sub = nh.subscribe("/odom", 1, odomCallback);
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/gmapping_odom", 1);
   
   ros::Rate rate(20.0);
   double x_t, y_t;
  
   //Instantiate a local listener
   echoListener echoListener;
 
 
    std::string source_frameid = "/map";
    std::string target_frameid = "/odom";
 
   // Wait for up to one second for the first transforms to become avaiable. 
   echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));
 
   //Nothing needs to be done except wait for a quit
   //The callbacks withing the listener class
   //will take care of everything
   while(nh.ok())
     {
       try
       {
         tf::StampedTransform echo_transform;
         echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
         std::cout.setf(std::ios::fixed,std::ios::floatfield);
         std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
         double yaw, pitch, roll;
         echo_transform.getBasis().getRPY(roll, pitch, yaw);
         tf::Quaternion q = echo_transform.getRotation();
         tf::Vector3 v = echo_transform.getOrigin();
         std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
         std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
                   << q.getZ() << ", " << q.getW() << "]" << std::endl
                   << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
                   << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;
 
         //print transform
         x_t = x*cos(yaw)-y*sin(yaw)+v.getX();
         y_t = x*sin(yaw)+y*cos(yaw)+v.getY()+1.0;
         std::cout << "- Odometry: [" << x << ", " << y << ", " << yaw_  << "]" << std::endl;
         std::cout << "- Position corrected by gmapping: [" << x_t << ", " << y_t << ", " << yaw_ + yaw << "]" << std::endl;
         
         
         geometry_msgs::Twist pos;
         pos.linear.x = x_t;
         pos.linear.y = y_t;
         pos.angular.z = yaw_ + yaw;
         pub.publish(pos);
         
       }
       catch(tf::TransformException& ex)
       {
         std::cout << "Failure at "<< ros::Time::now() << std::endl;
         std::cout << "Exception thrown:" << ex.what()<< std::endl;
         std::cout << "The current list of frames is:" <<std::endl;
         std::cout << echoListener.tf.allFramesAsString()<<std::endl;
         
       }
       ros::spinOnce();
       rate.sleep();
     }
 
   return 0;
 }
