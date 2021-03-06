#ifndef TOPIC_TOOLS_SHAPE_SHIFTER_H
#define TOPIC_TOOLS_SHAPE_SHIFTER_H

#ifdef _WIN32
   #define DECLSPEC _declspec(dllexport)
#elif __linux__
   #define DECLSPEC 
#else
#   error "Unknown compiler"
#endif

#include "ros/ros.h"
#include "ros/console.h"
#include "ros/assert.h"
#include <vector>
#include <string>
#include <string.h>

#include <ros/message_traits.h>
#include <topic_tools/macros.h>

namespace topic_tools
{
    class DECLSPEC ShapeShifterException : public ros::Exception
    {
        public:
            ShapeShifterException(const std::string& msg):ros::Exception(msg) {}
    };

    class DECLSPEC ShapeShifter
    {
        public:
            typedef boost::shared_ptr<ShapeShifter> Ptr;
            typedef boost::shared_ptr<ShapeShifter const> ConstPtr;

            static bool uses_old_API_;

            // Constructor and destructor
            ShapeShifter();
            virtual ~ShapeShifter();

            // Helpers for inspecting shapeshifter
            std::string const& getDataType()          const;
            std::string const& getMD5Sum()            const;
            std::string const& getMessageDefinition() const;

            void morph(const std::string& md5sum, const std::string& datatype, const std::string& msg_def, const std::string& latching);

            // Helper for advertising
            ros::Publisher advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_, bool latch=false, const ros::SubscriberStatusCallback &connect_cb=ros::SubscriberStatusCallback()) const;

            //! Call to try instantiating as a particular type
            template<class M> 
            boost::shared_ptr<M> instantiate() const;

            //! Write serialized message contents out to a stream
            template<typename Stream>
            void write(Stream& stream) const;

            template<typename Stream>
            void read(Stream& stream);

            //! Return the size of the serialized message
            uint32_t size() const;

        private:
            std::string md5, datatype, msg_def, latching;
            bool typed;

            uint8_t *msgBuf;
            uint32_t msgBufUsed;
            uint32_t msgBufAlloc;
    };
}


// Message traits allow shape shifter to work with the new serialization API
namespace ros 
{
    namespace message_traits 
    {

        template <> struct IsMessage<topic_tools::ShapeShifter> : TrueType { };
        template <> struct IsMessage<const topic_tools::ShapeShifter> : TrueType { };

        template<>
        struct MD5Sum<topic_tools::ShapeShifter>
        {
            static const char* value(const topic_tools::ShapeShifter& m) { return m.getMD5Sum().c_str(); }

            // Used statically, a shapeshifter appears to be of any type
            static const char* value() { return "*"; }
        };

        template<>
        struct DataType<topic_tools::ShapeShifter>
        {
            static const char* value(const topic_tools::ShapeShifter& m) { return m.getDataType().c_str(); }

            // Used statically, a shapeshifter appears to be of any type
            static const char* value() { return "*"; }
        };

        template<>
        struct Definition<topic_tools::ShapeShifter>
        {
            static const char* value(const topic_tools::ShapeShifter& m) { return m.getMessageDefinition().c_str(); }
        };

    } // namespace message_traits

    namespace serialization
    {

        template<>
        struct Serializer<topic_tools::ShapeShifter>
        {
            template<typename Stream>
            inline static void write(Stream& stream, const topic_tools::ShapeShifter& m) 
            {
                m.write(stream);
            }

            template<typename Stream>
            inline static void read(Stream& stream, topic_tools::ShapeShifter& m)
            {
                m.read(stream);
            }

            inline static uint32_t serializedLength(const topic_tools::ShapeShifter& m) 
            {
                return m.size();
            }
        };

        template<>
        struct PreDeserialize<topic_tools::ShapeShifter>
        {
            static void notify(const PreDeserializeParams<topic_tools::ShapeShifter>& params)
            {
                std::string md5      = (*params.connection_header)["md5sum"];
                std::string datatype = (*params.connection_header)["type"];
                std::string msg_def  = (*params.connection_header)["message_definition"];
                std::string latching  = (*params.connection_header)["latching"];

                params.message->morph(md5, datatype, msg_def, latching);
            }
        };
    } // namespace serialization
} //namespace ros



// Template implementations:

namespace topic_tools
{
    //  only used in testing, seemingly

    template<class M> 
    boost::shared_ptr<M> ShapeShifter::instantiate() const
    {
        if (!typed) throw ShapeShifterException("Tried to instantiate message from an untyped shapeshifter.");
        if (ros::message_traits::datatype<M>() != getDataType()) throw ShapeShifterException("Tried to instantiate message without matching datatype.");
        if (ros::message_traits::md5sum<M>() != getMD5Sum()) throw ShapeShifterException("Tried to instantiate message without matching md5sum.");
      
        boost::shared_ptr<M> p(boost::make_shared<M>());

        ros::serialization::IStream s(msgBuf, msgBufUsed);
        ros::serialization::deserialize(s, *p);

        return p;
    }

    template<typename Stream>
    void ShapeShifter::write(Stream& stream) const 
    {
        if (msgBufUsed > 0) memcpy(stream.advance(msgBufUsed), msgBuf, msgBufUsed);
    }

    template<typename Stream>
    void ShapeShifter::read(Stream& stream)
    {
        stream.getLength();
        stream.getData();
        
        // stash this message in our buffer
        if (stream.getLength() > msgBufAlloc)
        {
            delete[] msgBuf;
            msgBuf = new uint8_t[stream.getLength()];
            msgBufAlloc = stream.getLength();
        }

        msgBufUsed = stream.getLength();
        memcpy(msgBuf, stream.getData(), stream.getLength());
    }

} // namespace topic_tools


#endif

