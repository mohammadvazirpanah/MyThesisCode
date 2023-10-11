#ifndef ROS_HSDF_WRAPPERS_HPP
#define ROS_HSDF_WRAPPERS_HPP

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sy_process.hpp"
#include "abst_ext.hpp"
#include <stdio.h>
#include <math.h>

namespace ForSyDe
{

namespace SDF
{

using namespace sc_core;

template <typename T0, typename T1, typename T2>
class sdf_roswrap : public sdf_process

{
public:
    SDF_in<T1>  iport1;     
    SDF_out<T0> oport1;
    SDF_out<T2> oport2;


    sdf_roswrap(const sc_module_name& _name,                    
            const std::vector <std::string>& topics_publisher,          
            const std::vector <std::string>& topics_subscriber
            ): 
            sdf_process(_name),
            iport1("iport1"),
            oport1("oport1"),
            oport2("oport2"),
            topics_publisher(topics_publisher),
            topics_subscriber(topics_subscriber),
            first_run(true)

    {
#ifdef FORSYDE_INTROSPECTION
        //arg_vec.push_back(std::make_tuple("topic_name",topics_publisher, topics_subscriber));
#endif
    }

    std::string forsyde_kind() const {return "SDF::roswrap";}


private:

std::vector <std::string> topics_publisher;
std::vector <std::string> topics_subscriber;
std::string _name;
T1 *ival1; 
T0 *oval1;
T2 *oval2; 

ros::Publisher pub1;         
ros::Subscriber sub1, sub2;      
ros::NodeHandle *n;                       // Node Handle For Ros Environment 
ros::Rate *rate;                          // Ros Rate (Nedded For Timed Callbacks)
bool first_run;                           // First Run Flag


/* CallBack Functions are Declerated Here */
void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // ROS_INFO("Seq: [%d]", msg->header.seq);
  // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  // *oval1 = msg->pose.pose.position.x ;
}

  bool checkObjectInRange(const sensor_msgs::LaserScan::ConstPtr& scan, int start_idx, int end_idx, float threshold_distance) 
  {
      for (int i = start_idx; i <= end_idx; ++i) 
      {
          float range = scan->ranges[i];
          if (range > scan->range_min && range < scan->range_max && range < threshold_distance) 
          {
              return true; // Object detected within threshold distance
          }
      }
      return false; // No object detected within threshold distance
  }

void Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    
    std::array<float, 360> scan_array;
    std::vector<float> scan_vect;
    std::copy(scan->ranges.begin(), scan->ranges.end(), std::back_inserter(scan_vect));

    float threshold_distance = 1.5; 
 


    // Detect objects in 4 directions (front, back, left, right)
    bool object_front_1 = checkObjectInRange(scan, 0, 25, threshold_distance);
    bool object_front_2 = checkObjectInRange(scan, 335, 359, threshold_distance);
    bool object_front = object_front_1 || object_front_2;
    bool object_left = checkObjectInRange(scan, 25, 135, threshold_distance);
    bool object_back = checkObjectInRange(scan, 135, 225, threshold_distance);
    bool object_right = checkObjectInRange(scan, 225, 315, threshold_distance);
        
     // Print object detection status
    std::cout << "Object detected:\n";
    std::cout << "Front: " << (object_front ? "Yes" : "No") << std::endl;
    std::cout << "Back: " << (object_back ? "Yes" : "No") << std::endl;
    std::cout << "Left: " << (object_left ? "Yes" : "No") << std::endl;
    std::cout << "Right: " << (object_right ? "Yes" : "No") << std::endl;
    
    *oval2 = scan_vect;

}
  
  
    void init()
    {

      std::cout<<"ROS Wrapper Init"<<std::endl;
      ival1 = new T1;
      oval1 = new T0;
      oval2 = new T2;

      int argc;
      char **argv; 
		  ros::init(argc, argv, "ForSyDe_ROS_Wrapper");
      ros::start();
      n = new ros::NodeHandle;

      pub1 = n->advertise<geometry_msgs::Twist>(topics_publisher[0], 1000);

      sub1 = n->subscribe (topics_subscriber[0], 1000, &sdf_roswrap::Odom_Callback, this);
      sub2 = n->subscribe (topics_subscriber[1], 1000, &sdf_roswrap::Scan_Callback, this);
  
    }
    
    void prep()
    {
      std::cout<<"ROS Wrapper Prep"<<std::endl;

      if (first_run)
      {
        first_run = false;
        return;
      }
      
      *ival1 = iport1.read();


      // geometry_msgs::Twist msg1;

      // msg1.linear.x = *ival1;
      // msg1.linear.y = ival1->from_abst_ext(0.0);
      //  msg1.angular.z = ival1->from_abst_ext(0.0);

      
      // pub1.publish(msg1);

      
      ros::spinOnce(); 
      
    }
    
    void exec() 
    {

    }
    
    void prod()
    {
      std::cout<<"ROS Wrapper Prod"<<std::endl;
      
      while (ros::ok())

      {
        ros::spinOnce(); 
        wait(SC_ZERO_TIME);
      }

      write_multiport(oport1, *oval1);
      write_multiport(oport2, *oval2);



    }
    
    void clean()
    {
      delete ival1; 

      delete oval1;
      delete oval2;

      delete n;
      delete rate;
      ros::shutdown();
    }

    #ifdef FORSYDE_INTROSPECTION
    void bindInfo()
    {
        boundInChans.resize(3);     
        boundInChans[0].port = &iport1;

        boundOutChans.resize(6);    
        boundOutChans[0].port = &oport1;


    }
    #endif
};



template <class T0, template <class> class OIf,
          class T1, template <class> class I1If,
          class T2, template <class> class O2IF>
inline sdf_roswrap<T0,T1,T2>* make_sdf_roswrap(const std::string& pName,
    const std::vector <std::string>& pnames,
    const std::vector <std::string>& snames,
    OIf<T0>& out1S,
    O2IF<T2>& out2S, 
    I1If<T1>& inp1S
    )
{
    auto p = new sdf_roswrap<T0,T1,T2>(pName.c_str(), pnames, snames);
    
    (*p).iport1(inp1S);

    (*p).oport1(out1S);
    (*p).oport2(out2S);



    return p;
}

}
}

#endif
