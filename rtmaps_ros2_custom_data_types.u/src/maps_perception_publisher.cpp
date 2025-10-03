////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_perception_publisher.h"	// Includes the header of this component


// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSperception_publisher)
    MAPS_INPUT("exwayz_detections", MAPS::FilterStream8, MAPS::LastOrNextReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSperception_publisher)
    //MAPS_OUTPUT("oName",MAPS::Integer,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSperception_publisher)
    MAPS_PROPERTY("topic_name","/rtmaps/perception",false,false)
    MAPS_PROPERTY_ENUM("published_timestamps","RTMaps samples timestamps|ROS current time",1,false,false)
    MAPS_PROPERTY("frame_id","map",false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSperception_publisher)
    //MAPS_ACTION("aName",MAPSros_topic_publisher::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ros_topic_publisher) behaviour
MAPS_COMPONENT_DEFINITION(MAPSperception_publisher,"perception_publisher","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
              -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
              -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions


MAPSperception_publisher::MAPSperception_publisher(const char* name, MAPSComponentDefinition& cd) :
MAPSComponent(name, cd)
{
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

}

void MAPSperception_publisher::Birth()
{
    m_first_time = true;
	m_pub = nullptr;
	MAPSString topic_name = GetStringProperty("topic_name");


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
    m_pub = m_n->create_publisher< interface_rtmaps_msgs::msg::Perception >((const char*)topic_name, 100);

    m_count = 0;
}

void MAPSperception_publisher::Core()
{
    MAPSIOElt* ioeltin = StartReading(Input(0));
    if (ioeltin == nullptr)
    {
        return;
    }
    MAPSTimestamp t = ioeltin->Timestamp();

    int num_detections = ioeltin->VectorSize() / sizeof(Detection);
    if (num_detections >= MAX_OBJECTS)
    {
      num_detections = MAX_OBJECTS;
    }
      
    MAPSString frame_id = (const char*)GetStringProperty("frame_id");
    m_header.frame_id = frame_id.Len() > 0 ? (const char*)frame_id : (const char*)this->Name();
    if (m_publish_rtmaps_timestamp)
        m_header.stamp = MAPSRos2Utils::MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_n->now();

    using el_typo = interface_rtmaps_msgs::msg::Perception;
    auto msg = std::make_unique<el_typo>();
    Detection* det = (Detection*) ioeltin->Data();
    for (int i=0; i< num_detections; i++)    
    {
        interface_rtmaps_msgs::msg::Detection det_out;
        det_out.box_center[0] = (*det).box_center_[0];
        det_out.box_center[1] = (*det).box_center_[1];
        det_out.box_center[2] = (*det).box_center_[2];
        det_out.box_size[0]= (*det).box_size_[0];
        det_out.box_size[1]= (*det).box_size_[1];
        det_out.box_size[2]= (*det).box_size_[2];
        det_out.box_angle = (*det).box_angle_;
        det_out.centroid[0] = (*det).centroid_[0];
        det_out.centroid[1] = (*det).centroid_[1];
        det_out.centroid[2] = (*det).centroid_[2];
        det_out.velocity[0] = (*det).velocity_[0];
        det_out.velocity[1] = (*det).velocity_[1];
        det_out.velocity[2] = (*det).velocity_[2];
        det_out.num_pts = (*det).num_pts_;
        det_out.age = (*det).age_;
        det_out.objectid = (*det).objectid_;
        det_out.classid = (*det).classid_;

        msg->detections.push_back(det_out);

        det++;
    }
    msg->header = m_header;

    auto p = (rclcpp::Publisher<el_typo>*) m_pub.get();
    p->publish(std::move(msg));

    m_count++;
}

void MAPSperception_publisher::Death()
{
    m_pub = nullptr;
    //m_n = nullptr;
}
