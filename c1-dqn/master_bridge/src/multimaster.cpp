#include "master_bridge/multimaster.h"

multimaster::multimaster() { }
multimaster::~multimaster() { }

std::string multimaster::foreign_master_uri()
{
    return foreign_master;
}

std::vector<std::string> multimaster::get_topic(XmlRpc::XmlRpcValue L)
{
    std::vector<std::string> rtn;
    if ((L.getType() == XmlRpc::XmlRpcValue::TypeArray) && (L.size() > 0))
    {
        for (int i = 0; i < L.size(); ++i)
        {
            rtn.push_back(static_cast<std::string>(L[i]));
        }
    }
    return rtn;
}

std::vector<std::vector<std::string>> multimaster::get_tf(XmlRpc::XmlRpcValue L)
{
    std::vector<std::vector<std::string>> rtn;
    std::vector<std::string> ele;
    XmlRpc::XmlRpcValue point;

    for (int i = 0; i < L.size(); ++i)
    {
        point = L[i];
        if (point.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            ele.assign(2, "");
            if(point.hasMember("to"))
            {
                ele[0]= static_cast<std::string>(point["to"]);
            }
            if(point.hasMember("from"))
            {
                ele[1] = static_cast<std::string>(point["from"]);
            }
            rtn.push_back(ele);
        }
    }

    return rtn;
}

void multimaster::getPm()
{
    ros::NodeHandle nh_param("~");

    // Get the other parameters from the launch file
    if (!nh_param.getParam("foreign_ip", foreign_ip)) foreign_ip = "localhost"; 
    if (!nh_param.getParam("foreign_port", foreign_port)) foreign_port = 11311; 
    if (!nh_param.getParam("connectCheck_Hz", connectCheck_Hz)) connectCheck_Hz = 200;
    if (!nh_param.getParam("msgsFrequency_Hz", msgsFrequency_Hz)) msgsFrequency_Hz = 10; 
    if (!nh_param.getParam("namespace", namesp)) namesp = ""; 
    if (!nh_param.getParam("pub_queue_size", pub_queue_size)) pub_queue_size = 10;
    if (!nh_param.getParam("pub_name", pub_name)) pub_name = "master_bridge";

    if (nh_param.getParam("TopicList", hostTopicsList_rpcxml)) hostTopicsList = get_topic(hostTopicsList_rpcxml);
    if (nh_param.getParam("TfList", hostTfList_rpcxml)) hostTfList = get_tf(hostTfList_rpcxml);

    //Get the host and foreign master_uri
    std::stringstream foreign_master_uri;
    foreign_master_uri << "http://" << foreign_ip << ":" << foreign_port << "/";
    foreign_master = foreign_master_uri.str();
    host_master = ros::master::getURI();
}

void multimaster::host2foreign(ros::M_string remappings) 
{
    //ros::Time before = ros::Time::now();
    //ros::Duration difference;

    remappings["__master"] = host_master;
    remappings["__name"] = "master_bridge";
    ros::master::init(remappings);

    relayTopic pc;
    pc.set_queue_size(pub_queue_size);
    for(int hostTopicNum=0 ; hostTopicNum< hostTopicsList.size() ; hostTopicNum++)
    {         
        //Create subscribers in the host and connect them to the foreign topics 
        pc.subscribe(hostTopicsList[hostTopicNum], namesp, nh); 
        std::cout<<"hostTopicsList : " << hostTopicsList[hostTopicNum].c_str() << "- Fine" << "\n";
    }

    remappings["__master"] = foreign_master;
    remappings["__name"] = pub_name;
    ros::master::init(remappings);

    float connectCheck_cycle = 1.0/(float)connectCheck_Hz;
    ros::Duration(connectCheck_cycle).sleep();

    ros::Rate loop_rate(msgsFrequency_Hz); 

    while(ros::ok() && ros::master::check()==true)
    {
        ros::spinOnce();
        loop_rate.sleep();

        tf::TransformBroadcaster broadcaster;
        for(int hostTfNum = 0; hostTfNum < hostTfList.size(); hostTfNum++) 
        {
            //difference = ros::Time::now() - before;
            //broadcaster.sendTransform(tf::StampedTransform(pc.listen(ros::Time::now() + difference, hostTfList[hostTfNum][1], hostTfList[hostTfNum][0])));
            broadcaster.sendTransform(tf::StampedTransform(pc.listen(ros::Time::now(), hostTfList[hostTfNum][1], hostTfList[hostTfNum][0])));
        }
    }
}

std::string multimaster::getForeign_ip()
{
    return foreign_ip;
}

int multimaster::getForeign_port()
{
    return foreign_port;
}

int multimaster::getConnectCheck_Hz()
{
    return connectCheck_Hz;
}

int multimaster::getMsgsFrequency_Hz()
{
    return msgsFrequency_Hz;
}

std::string multimaster::getNamespace()
{
    return namesp;
}

std::vector<std::string> multimaster::getTopicsList()
{
    return hostTopicsList;
}

std::vector<std::vector<std::string>> multimaster::getTfList()
{
    return hostTfList;
}

int multimaster::getPubQueueSize()
{
    return pub_queue_size;
}

std::string multimaster::getPubName()
{
    return pub_name;
}

