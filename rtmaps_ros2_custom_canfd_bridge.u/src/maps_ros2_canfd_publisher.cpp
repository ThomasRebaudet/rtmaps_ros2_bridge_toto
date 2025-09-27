////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_ros2_canfd_publisher.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPS_ros2_canfd_publisher)
    MAPS_INPUT("input",MAPS::FilterCANFDFrame,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPS_ros2_canfd_publisher)
    //MAPS_OUTPUT("oName",MAPS::Integer,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPS_ros2_canfd_publisher)
    MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY_ENUM("published_timestamps","RTMaps samples timestamps|ROS current time",1,false,false)
    MAPS_PROPERTY("frame_id","map",false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPS_ros2_canfd_publisher)
    //MAPS_ACTION("aName",MAPSros_topic_publisher::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ros_topic_publisher) behaviour
MAPS_COMPONENT_DEFINITION(MAPS_ros2_canfd_publisher,"ros2_canfd_publisher","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
              -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
              -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions



void MAPS_ros2_canfd_publisher::Birth()
{
	m_pub = nullptr;
	MAPSString topic_name = GetStringProperty("topic_name");

    try 
    {
        m_n = MAPSRos2Utils::GetROS2Node();
        if (m_n == nullptr)
            Error("Could not create NodeHandle.");
    }
    catch (std::exception& e)
    {
        MAPSStreamedString ss;
        ss << "Could not init ROS2: " << e.what();
        Error(ss);
    }

    m_pub = nullptr;

    int pub_ts = (int)GetIntegerProperty("published_timestamps");

    if (pub_ts == 0)
    {
        m_publish_rtmaps_timestamp = true;
    }
    else 
    {
        m_publish_rtmaps_timestamp = false;
    }
    m_pub = m_n->create_publisher<canfd_msgs::msg::FdFrame>((const char*)topic_name, 100);
}

void MAPS_ros2_canfd_publisher::Core()
{
	MAPSTimestamp t;
    MAPS::InputGuard<MAPSCANFDFrame> ig_input(this, Input(0));
    t = ig_input.Timestamp();

    MAPSString frame_id = (const char*)GetStringProperty("frame_id");
    m_header.frame_id = frame_id.Len() > 0 ? (const char*)frame_id : (const char*)this->Name();
    if (m_publish_rtmaps_timestamp)
        m_header.stamp = MAPSRos2Utils::MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_n->now();

    PublishMyMsg(ig_input);
}

void MAPS_ros2_canfd_publisher::PublishMyMsg(MAPS::InputGuard<MAPSCANFDFrame>& ig_input)
{
    using el_typo = canfd_msgs::msg::FdFrame;
    auto msg = std::make_unique< el_typo >();
    msg->id = ig_input.Data().GetId();
    msg->header = m_header;
    msg->len = ig_input.Data().dataLength;
    msg->is_extended = ig_input.Data().HasExtendedId();

    for(int i = 0; i < msg->len; ++i)
    {
        msg->data[i] = ig_input.Data().data[i];
    }

    auto p = (rclcpp::Publisher < el_typo > *) m_pub.get();
    p->publish(std::move(msg));

}

void MAPS_ros2_canfd_publisher::Death()
{
    m_pub = nullptr;
    m_n = nullptr;
}
