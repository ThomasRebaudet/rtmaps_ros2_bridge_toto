////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_my_ros2_datatype_publisher.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSmy_ros2_datatype_publisher)
    MAPS_INPUT("id",MAPS::FilterInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSmy_ros2_datatype_publisher)
    //MAPS_OUTPUT("oName",MAPS::Integer,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSmy_ros2_datatype_publisher)
    MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY_ENUM("published_timestamps","RTMaps samples timestamps|ROS current time",1,false,false)
    MAPS_PROPERTY("frame_id","map",false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSmy_ros2_datatype_publisher)
    //MAPS_ACTION("aName",MAPSros_topic_publisher::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ros_topic_publisher) behaviour
MAPS_COMPONENT_DEFINITION(MAPSmy_ros2_datatype_publisher,"my_ros2_datatype_publisher","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
              -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
              -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions



void MAPSmy_ros2_datatype_publisher::Birth()
{
    m_first_time = true;
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
    m_pub = m_n->create_publisher< my_data_type::msg::MyDataType >((const char*)topic_name, 100);

	m_count = 0;
}

void MAPSmy_ros2_datatype_publisher::Core()
{
	MAPSTimestamp t;
    m_ioeltin = StartReading(Input(0));
    if (m_ioeltin == nullptr)
        return;
    t = m_ioeltin->Timestamp();

    MAPSString frame_id = (const char*)GetStringProperty("frame_id");
    m_header.frame_id = frame_id.Len() > 0 ? (const char*)frame_id : (const char*)this->Name();
    if (m_publish_rtmaps_timestamp)
        m_header.stamp = MAPSRos2Utils::MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_n->now();

    PublishMyMsg();

    m_count++;
}

void MAPSmy_ros2_datatype_publisher::PublishMyMsg()
{
    using el_typo = my_data_type::msg::MyDataType;
    auto msg = std::make_unique< el_typo >();
    msg->id = m_ioeltin->Integer32();
    msg->header = m_header;

    auto p = (rclcpp::Publisher < el_typo > *) m_pub.get();
    p->publish(std::move(msg));

}

void MAPSmy_ros2_datatype_publisher::Death()
{
    m_pub = nullptr;
    m_n = nullptr;
}
