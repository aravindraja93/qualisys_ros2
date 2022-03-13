#include <unistd.h>
#include <math.h>
#include <map>
#include <cstdio>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "RTProtocol.h"
#include "RTPacket.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class QualisysNode : public rclcpp::Node
{
    public:
    QualisysNode()
    : Node("qualisys_node")
    {   
         
        this->declare_parameter("server", "192.168.254.1");
        this->declare_parameter("rate_limit", 100.0);

        

        timer_ = this->create_wall_timer(
        500ms, std::bind(&QualisysNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "NODE INITIATED ");
        try
        {   //RCLCPP_INFO(this->get_logger(), "INSIDE____________________");
            rtProtocol.DiscoverRTServer(4534, false);
            //RCLCPP_INFO(this->get_logger(), "INSIDE");

            //Example code for how to use discovery calls.
            if (rtProtocol.DiscoverRTServer(4534, false))
            {
            //sleep(1);
            const auto numberOfResponses = rtProtocol.GetNumberOfDiscoverResponses();
            std::cout<<numberOfResponses<<std::endl;
            for (auto index = 0; index < numberOfResponses; index++)
            {
                unsigned int addr;
                
                std::string message;
                if (rtProtocol.GetDiscoverResponse(index, addr, basePort, message))
                {
                    printf("%2d - %d.%d.%d.%d:%d\t- %s\n", index, 0xff & addr, 0xff & (addr >> 8), 0xff & (addr >> 16), 0xff & (addr >> 24), basePort, message.c_str());
                }
            }
            }
            else
            {
            
            printf("%s", rtProtocol.GetErrorString());
            }

            basePort = 22222;
        
           
        }
        catch (std::exception &e)
        {
            
            printf("%s\n", e.what());
        }
    
        
       
    }

    // ~QualisysNode()
    // {
    //   printf("NODE SHUTDOWN 1 /n");
    //   rtProtocol.StreamFramesStop();
    //   rtProtocol.Disconnect();
    // }

    private:

    CRTProtocol rtProtocol;
    unsigned short basePort;  // find a way for const
    const int majorVersion = 1;
    const int minorVersion = 19;
    const bool bigEndian = false;

    bool dataAvailable = false;
    bool streamFrames = false;
    unsigned short udpPort = 6734;
    

    rclcpp::TimerBase::SharedPtr timer_;

    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_pose;
    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> pub_odom;
    std::map<std::string, rclcpp::Time> pub_stamps;

    std::string server;
    double rate_limit;
    int slow_count = 0; // watch for slow publication

    const int queue_size = 1;

    void timer_callback()
    {

        this->get_parameter("server", server);
        this->get_parameter("rate_limit", rate_limit);

            if (!rtProtocol.Connected())
            {
                if (!rtProtocol.Connect(server.c_str(), basePort, &udpPort, majorVersion, minorVersion, bigEndian))
                {
                    RCLCPP_INFO(this->get_logger(),"rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    return;
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol.Read6DOFSettings(dataAvailable))
                {
                    RCLCPP_INFO(this->get_logger(),"rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    return;
                }
            }

            if (!streamFrames)
            {
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d))
                {
                    RCLCPP_INFO(this->get_logger(),"rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    return;
                }
                streamFrames = true;

                RCLCPP_INFO(this->get_logger(),"Starting to streaming 6DOF data");
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol.ReceiveRTPacket(packetType, true) > 0)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket *rtPacket = rtProtocol.GetRTPacket();

                    //ROS_WARN("Frame %d\n", rtPacket->GetFrameNumber());

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            std::string name(rtProtocol.Get6DOFBodyName(i));
                            //ROS_WARN("data received for rigid body %s", name.c_str());

                            if (!isfinite(fX) || !isfinite(fY) || !isfinite(fZ)) {
                                RCLCPP_INFO(this->get_logger(), "rigid body %s tracking lost", name.c_str()); //WARN
                                continue;
                            }

                            for (int i=0; i<9; i++) {
                                if (!isfinite(rotationMatrix[i])) {
                                    RCLCPP_INFO(this->get_logger(), "rigid body %s tracking lost", name.c_str()); //WARN
                                    continue;
                                }
                            }

                            rclcpp::Time now = rclcpp::Node::now();

                            // convert to quaternion
                            tf2::Matrix3x3 R(
                              rotationMatrix[0], rotationMatrix[3], rotationMatrix[6],
                              rotationMatrix[1], rotationMatrix[4], rotationMatrix[7],
                              rotationMatrix[2], rotationMatrix[5], rotationMatrix[8]);
                            tf2::Quaternion q;
                            R.getRotation(q);

                            // scale position to meters from mm
                            double x = fX/1.0e3;
                            double y = fY/1.0e3;
                            double z = fZ/1.0e3;
                            double elapsed = 0;

                            // publish data if rate limit met
                            if (pub_stamps.count(name) == 0) {
                                elapsed = 0;
                            } else {
                              elapsed = (now - pub_stamps[name]).seconds();
                              if (elapsed < 0.99/rate_limit) {
                                // wait
                                continue;
                              }
                            }
                            pub_stamps[name] = now;

                            // warning if slow
                            if (elapsed > 3.0/rate_limit) {
                                slow_count += 1;
                                if (slow_count > 10) {
                                    RCLCPP_INFO(this->get_logger(), "publication rate low: %10.4f Hz", 1.0/elapsed);
                                    slow_count = 0;
                                }
                            }

                            // publish transform
                            {
                                std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
                                geometry_msgs::msg::TransformStamped transformStamped;
                                transformStamped.header.stamp = now;
                                transformStamped.header.frame_id = "qualisys";
                                transformStamped.child_frame_id = name;
                                transformStamped.transform.translation.x = x;
                                transformStamped.transform.translation.y = y;
                                transformStamped.transform.translation.z = z;
                                transformStamped.transform.rotation.x = q.x();
                                transformStamped.transform.rotation.y = q.y();
                                transformStamped.transform.rotation.z = q.z();
                                transformStamped.transform.rotation.w = q.w();
                                tf_broadcaster_->sendTransform(transformStamped);
                            }

                            // publish pose stamped message
                            {
                                if (pub_pose.find(name) == pub_pose.end()) {
                                    RCLCPP_INFO(this->get_logger(),"rigid body %s pose added", name.c_str());
                                    pub_pose[name] = this->create_publisher<geometry_msgs::msg::PoseStamped>(name + "/pose", queue_size);
                                    
                                }
                                // std::cout<<x<<" "<<y<<" "<<z<<std::endl;
                                geometry_msgs::msg::PoseStamped msg;
                                msg.header.frame_id="qualisys";
                                msg.header.stamp = now;
                                msg.pose.position.x = x;
                                msg.pose.position.y = y;
                                msg.pose.position.z = z;
                                msg.pose.orientation.x = q.x();
                                msg.pose.orientation.y = q.y();
                                msg.pose.orientation.z = q.z();
                                msg.pose.orientation.w = q.w();
                                pub_pose[name]->publish(msg);
                            }

                            // publish odom message
                            {
                                if (pub_odom.find(name) == pub_odom.end()) {
                                    RCLCPP_INFO(this->get_logger(),"rigid body %s odom added", name.c_str());
                                    pub_odom[name] = this->create_publisher<nav_msgs::msg::Odometry>(name + "/odom", queue_size);
                                }
                                // std::cout<<x<<" "<<y<<" "<<z<<std::endl;
                                nav_msgs::msg::Odometry msg;
                                msg.header.frame_id="qualisys";
                                msg.header.stamp = now;
                                msg.child_frame_id=name;
                                for (int i=0; i < 36; i++) msg.pose.covariance[i] = NAN;
                                msg.pose.pose.position.x = x;
                                msg.pose.pose.position.y = y;
                                msg.pose.pose.position.z = z;
                                msg.pose.pose.orientation.x = q.x();
                                msg.pose.pose.orientation.y = q.y();
                                msg.pose.pose.orientation.z = q.z();
                                msg.pose.pose.orientation.w = q.w();
                                for (int i=0; i < 36; i++) msg.twist.covariance[i] = NAN;
                                msg.twist.twist.linear.x = NAN;
                                msg.twist.twist.linear.y = NAN;
                                msg.twist.twist.linear.z = NAN;
                                msg.twist.twist.angular.x = NAN;
                                msg.twist.twist.angular.y = NAN;
                                msg.twist.twist.angular.z = NAN;
                                pub_odom[name]->publish(msg);
                            }
                        }
                    }
                }
            }
        }

    };
    




int main(int argc, char * argv[])
{
 
  rclcpp::init(argc, argv);
 
  rclcpp::spin(std::make_shared<QualisysNode>());  
  rclcpp::shutdown();

 

  return 0;
}
