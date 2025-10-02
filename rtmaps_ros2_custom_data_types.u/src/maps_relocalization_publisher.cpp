////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////


#define NB_INPUTS 15

#include "maps_relocalization_publisher.h"	// Includes the header of this component


// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSrelocalization_publisher)
    MAPS_INPUT("timestamp", MAPS::FilterOneFloat64, MAPS::FifoReader)
    MAPS_INPUT("state", MAPS::FilterOneUnsignedInteger8, MAPS::FifoReader)
    MAPS_INPUT("state_human_readable", MAPS::FilterTextUTF8, MAPS::FifoReader)
    MAPS_INPUT("pose", MAPS::FilterMatrix, MAPS::FifoReader)
    MAPS_INPUT("pose_cov", MAPS::FilterMatrix, MAPS::FifoReader)
    MAPS_INPUT("pose_std", MAPS::FilterFloats32, MAPS::FifoReader)
    MAPS_INPUT("lla", MAPS::FilterFloats64, MAPS::FifoReader)
    MAPS_INPUT("velocity", MAPS::FilterFloats32, MAPS::FifoReader)
    MAPS_INPUT("twist", MAPS::FilterFloats32, MAPS::FifoReader)
    MAPS_INPUT("overlap", MAPS::FilterOneFloat32, MAPS::FifoReader)
    MAPS_INPUT("overlap_static", MAPS::FilterOneFloat32, MAPS::FifoReader)
    MAPS_INPUT("processing_time", MAPS::FilterOneFloat32, MAPS::FifoReader)
    MAPS_INPUT("distance_to_map", MAPS::FilterOneFloat32, MAPS::FifoReader)
    MAPS_INPUT("num_pts_frame", MAPS::FilterOneInteger32, MAPS::FifoReader)
    MAPS_INPUT("num_pts_icp", MAPS::FilterOneInteger32, MAPS::FifoReader)
    //Adapt the above #define NB_INPUTS if you change the number of inputs.
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSrelocalization_publisher)
    //MAPS_OUTPUT("oName",MAPS::Integer,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSrelocalization_publisher)
    MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY_ENUM("published_timestamps","RTMaps samples timestamps|ROS current time",1,false,false)
    MAPS_PROPERTY("frame_id","map",false,false)
    MAPS_PROPERTY_SUBTYPE("inputs_synch_tolerance",100000,false,false,MAPS::PropertySubTypeTime)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSrelocalization_publisher)
    //MAPS_ACTION("aName",MAPSros_topic_publisher::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ros_topic_publisher) behaviour
MAPS_COMPONENT_DEFINITION(MAPSrelocalization_publisher,"relocalization_publisher","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
              -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
              -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions



// int MAPSrelocalization_publisher::countInputs() {
//     int count = 0;
//     try {
//         while(Input(count) != nullptr) {
//             count++;
//         }
//     } catch(...) {
//         // Fin des inputs atteinte
//     }
//     return count;
// }

void MAPSrelocalization_publisher::Birth()
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
    m_pub = m_n->create_publisher< interface_rtmaps_msgs::msg::Relocalization >((const char*)topic_name, 100);

	//m_nbInputs = countInputs();
    
    for (int i=0; i<NB_INPUTS; i++)
    {
        m_inputs[i] = &Input(i);
    }

    m_count = 0;
}

void MAPSrelocalization_publisher::Core()
{
    // Initialiser le vecteur avec tous les inputs
    /*
    m_inputs.clear();
    for(int i = 0; i < 9; i++) {
        MAPSInput& a = Input(i);
        MAPSIOElt* ioelt = StartReading(a);
        if (ioelt == nullptr) {
            return;
        }
        m_inputs.push_back(ioelt);
    }
    */


    // Utiliser le timestamp du premier input
    //t = m_inputs[0]->Timestamp();

    MAPSTimestamp t = SynchroStartReading(NB_INPUTS,m_inputs,m_ioelts, GetIntegerProperty(3));
    if (t == -1)
        return;

    MAPSString frame_id = (const char*)GetStringProperty("frame_id");
    m_header.frame_id = frame_id.Len() > 0 ? (const char*)frame_id : (const char*)this->Name();
    if (m_publish_rtmaps_timestamp)
        m_header.stamp = MAPSRos2Utils::MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_n->now();

    PublishMyMsg();

    // Libérer tous les inputs
    
    //m_inputs.clear();

    m_count++;
}

void MAPSrelocalization_publisher::PublishMyMsg()
{
    using el_typo = interface_rtmaps_msgs::msg::Relocalization;
    auto msg = std::make_unique<el_typo>();
    
    // Correct access to input data
    msg->timestamp = m_ioelts[0]->Float64();  // For FilterOneFloat64
    msg->state = m_ioelts[1]->Stream8()[0];  // For FilterUnsignedInteger8
    msg->state_human_readable = m_ioelts[2]->Text();  // For FilterTextUTF8

    // For arrays, use Float64() and Float32() with & operator to get address
    MAPSMatrix& pose_mtrx = m_ioelts[3]->Matrix();
    MAPSMatrix& pose_cov = m_ioelts[4]->Matrix();
    const float* pose_std_ptr = &m_ioelts[5]->Float32(0);
    const double* lla_ptr = &m_ioelts[6]->Float64(0);
    const float* velocity_ptr = &m_ioelts[7]->Float32(0);
    const float* twist_ptr = &m_ioelts[8]->Float32(0);


    // Copy data using memcpy with proper pointers
    memcpy(msg->pose.data(), pose_mtrx.pr, 16 * sizeof(double));
    memcpy(msg->pose_cov.data(), pose_cov.pr, 36 * sizeof(double));
    memcpy(msg->pose_std.data(), pose_std_ptr, 6 * sizeof(float));
    memcpy(msg->lla.data(), lla_ptr, 3 * sizeof(double));
    memcpy(msg->velocity.data(), velocity_ptr, 6 * sizeof(float));
    memcpy(msg->twist.data(), twist_ptr, 6 * sizeof(float));
    
    msg->overlap = m_ioelts[9]->Float32();
    msg->overlap_static = m_ioelts[10]->Float32();
    msg->processing_time = m_ioelts[11]->Float32();
    msg->distance_to_map = m_ioelts[12]->Float32();
    msg->num_pts_frame = m_ioelts[13]->Integer32();
    msg->num_pts_icp = m_ioelts[14]->Integer32();   

    msg->header = m_header;

    auto p = (rclcpp::Publisher<el_typo>*) m_pub.get();
    p->publish(std::move(msg));
}

void MAPSrelocalization_publisher::Death()
{
    m_pub = nullptr;
    m_n = nullptr;
}