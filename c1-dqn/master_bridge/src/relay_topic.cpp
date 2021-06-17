#include "master_bridge/relay_topic.h"

//class which create  multimaster/chatter on the foreign pc
relayTopic::relayTopic() { queue_size = 10; }
relayTopic:: ~ relayTopic() { }

void relayTopic::callback(const ros::MessageEvent < topic_tools::ShapeShifter >& msg_event, std::string & topic) 
{
    std::string publisher_name = msg_event.getPublisherName();
    boost::shared_ptr <topic_tools::ShapeShifter const> const & msg = msg_event.getConstMessage();
    boost::shared_ptr <const ros::M_string> const & connection_header = msg_event.getConnectionHeaderPtr();
    ros::Publisher publ = relayTopic::getPublisher(namesp_ + "/" + topic, msg, connection_header);
    publ.publish(msg);
}

void relayTopic::set_queue_size(int qs)
{
    queue_size = qs;
}

ros::Publisher relayTopic::getPublisher(const std::string & topic, boost::shared_ptr<topic_tools::ShapeShifter const> const & msg, boost::shared_ptr < const ros::M_string > const & connection_header)
{
    if (mPublishers.find(topic) == mPublishers.end()) 
    {
        bool latch = false;
        // If the input topic is latched, make the output topic latched.
        if (connection_header) 
        {
            ros::M_string::const_iterator it = connection_header->find("latching");
            if ((it != connection_header->end()) && (it->second == "1")) 
            {
                ROS_DEBUG("input topic is latched; latching output topic to match");
                latch = true;
            }
        }
        //advertise new topic
        mPublishers[topic] = msg->advertise(n, topic, queue_size, latch);
    }
    return mPublishers[topic];
}

void relayTopic::subscribe(std::string g_input_topic, std::string namesp, ros::NodeHandle nh) 
{
    ros::Subscriber subscriber = nh.subscribe<topic_tools::ShapeShifter> (g_input_topic, queue_size, boost::bind(& relayTopic::callback, this, _1, g_input_topic));
    subs.push_back(subscriber);
    namesp_ = namesp;
}

tf::StampedTransform relayTopic::listen(ros::Time time, std::string from,std::string to) 
{
    // tf::TransformListener listener; tf::StampedTransform transform;
    try 
    {
        listener.waitForTransform(from, to, time, ros::Duration(3.0));
        listener.lookupTransform(from, to, time, transform);
        transform.stamp_ = ros::Time::now();
    }
    catch (tf::TransformException ex) 
    {
        ROS_ERROR("%s", ex.what());
        //ros::Duration(1.0).sleep();
    }
    return transform;
}
