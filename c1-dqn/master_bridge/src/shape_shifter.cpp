
#include <master_bridge/shape_shifter.h> 

using namespace topic_tools;

bool ShapeShifter::uses_old_API_ = false;

ShapeShifter::ShapeShifter(): typed(false), msgBuf(NULL), msgBufUsed(0), msgBufAlloc(0) {}
ShapeShifter::~ShapeShifter() 
{
    if (msgBuf) delete[] msgBuf;
    msgBuf = NULL;
    msgBufAlloc = 0;
}

std::string const & ShapeShifter::getDataType() const 
{
    return datatype;
}

std::string const & ShapeShifter::getMD5Sum() const 
{
    return md5;
}

std::string const & ShapeShifter::getMessageDefinition() const 
{
    return msg_def;
}

void ShapeShifter::morph(const std::string & _md5sum, const std::string & _datatype, const std::string & _msg_def, const std::string & _latching) 
{
    md5 = _md5sum;
    datatype = _datatype;
    msg_def = _msg_def;
    latching = _latching;
    typed = md5 != "*";
}

ros::Publisher ShapeShifter::advertise(ros::NodeHandle & nh, const std::string & topic, uint32_t queue_size_, bool latch, const ros::SubscriberStatusCallback & connect_cb) const 
{
    ros::AdvertiseOptions opts(topic, queue_size_, getMD5Sum(), getDataType(), getMessageDefinition(), connect_cb);
    opts.latch = latch;
    return nh.advertise(opts);
}

uint32_t ShapeShifter::size() const 
{
    return msgBufUsed;
}
