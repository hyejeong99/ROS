#ifndef MULTIMASTER_H
#define	MULTIMASTER_H

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include "master_bridge/relay_topic.h"

namespace ros {
    namespace master 
    {
        void init(const M_string& remappings);
    }
}

//The class with functions which read the parameters from launch file and subscribe topics
class multimaster 
{
    public:
        multimaster();
        ~multimaster();
        
        void getPm();
        void init(ros::M_string remappings);
        void host2foreign(ros::M_string remappings);
        //void init_pc();

        int getForeign_port();
        int getConnectCheck_Hz();
        int getMsgsFrequency_Hz();
        int getPubQueueSize();

        std::string getForeign_ip();
        std::string getNamespace();
        std::string getPubName();
        
        std::vector<std::string> getTopicsList();
        std::vector<std::vector<std::string>> getTfList();

        std::string foreign_master_uri();

    private:
        // Func
        std::vector<std::string> get_topic(XmlRpc::XmlRpcValue L);
        std::vector<std::vector<std::string>> get_tf(XmlRpc::XmlRpcValue L);

        // Var
        ros::NodeHandle nh;

        XmlRpc::XmlRpcValue hostTfList_rpcxml;
        XmlRpc::XmlRpcValue hostTopicsList_rpcxml;

        std::vector<std::string> hostTopicsList;
        std::vector<std::vector<std::string>> hostTfList;
        
        std::string namesp;
        std::string host_master;
        std::string foreign_master; 
        std::string foreign_ip;
        std::string pub_name;

        int foreign_port; 
        int msgsFrequency_Hz;
        int connectCheck_Hz;
        int pub_queue_size;
};

#endif	/* MULTIMASTER_H */
