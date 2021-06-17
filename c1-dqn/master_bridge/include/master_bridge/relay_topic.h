#ifndef RELAY_TOPIC_H
#define RELAY_TOPIC_H

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/Empty.h"

#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/package.h>
#include <cstdio>
#include <XmlRpcValue.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class relayTopic
{
    public:
        relayTopic();
        ~relayTopic();

        //tf listener
        tf::TransformListener listener;
        tf::StampedTransform transform;
        
        void subscribe(std::string g_input_topic,std::string namesp, ros::NodeHandle nh);//generic subscriber
        void callback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event, std::string& topic);//generic callback function which also create generic publisher
        tf::StampedTransform listen(ros::Time time, std::string from, std::string to);
        ros::Publisher getPublisher(const std::string& topic,  boost::shared_ptr<topic_tools::ShapeShifter const> const &msg, boost::shared_ptr<const ros::M_string> const& connection_header);
        void set_queue_size(int qs);

    private:
        std::map<std::string, ros::Publisher> mPublishers;
        std::vector<ros::Subscriber> subs; 
        std::vector<std::string> topics; 
        ros::NodeHandle *g_node = NULL;
        ros::NodeHandle n;
        std::string namesp_;
        int queue_size;
};

#endif	/* RELAY_TOPIC_H */

