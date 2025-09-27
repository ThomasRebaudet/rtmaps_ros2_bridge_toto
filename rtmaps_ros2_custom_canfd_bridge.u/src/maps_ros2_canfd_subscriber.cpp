////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_ros2_canfd_subscriber.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPS_ros2_canfd_subscriber)
    //MAPS_INPUT("iName",MAPS::FilterInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPS_ros2_canfd_subscriber)
    MAPS_OUTPUT("output",MAPS::CANFDFrame,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPS_ros2_canfd_subscriber)
	MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY("transfer_ROS_timestamps",true,false,false)
    MAPS_PROPERTY("subscribe_queue_size",-1,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPS_ros2_canfd_subscriber)
    //MAPS_ACTION("aName",MAPSmy_data_types_component::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (my_data_types_component) behaviour
MAPS_COMPONENT_DEFINITION(MAPS_ros2_canfd_subscriber,"ros2_canfd_subscriber","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions

//Initialization: Birth() will be called once at diagram execution startup.			  
void MAPS_ros2_canfd_subscriber::Birth()
{
    m_first_time = true;
    m_transfer_ros_timestamp = GetBoolProperty("transfer_ROS_timestamps");

    try 
    {
        m_n = MAPSRos2Utils::GetROS2Node();
        if (m_n == nullptr)
            Error("Could not create NodeHandle.");
    }
    catch (std::exception& e)
    {
        Error(e.what());
    }

    m_sub = NULL;

    int queue_size = (int)GetIntegerProperty("subscribe_queue_size");

    MAPSString topic_name = GetStringProperty("topic_name");

    try
    {
    using el_typo = canfd_msgs::msg::FdFrame;
    queue_size = queue_size == -1?1000:queue_size;
    std::function<void(std::shared_ptr < el_typo > )> fnc = std::bind(
            &MAPS_ros2_canfd_subscriber::ROSDataReceivedCallback, this, std::placeholders::_1);
    m_sub = m_n->create_subscription<el_typo>((const char *) topic_name, queue_size, fnc);
    }
    catch (std::exception& e)
    {
        Error(e.what());
    }

    if (m_sub == NULL) 
    {
		MAPSStreamedString ss;
        ss << "Could not subscribe to topic " << topic_name;
		Error(ss);
	}
}

void MAPS_ros2_canfd_subscriber::ROSDataReceivedCallback(const canfd_msgs::msg::FdFrame::ConstSharedPtr& message)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();

        MAPS::OutputGuard<MAPSCANFDFrame> frameOut(this, Output(0));
        frameOut.Data().arbitrationId = message->id;
        frameOut.Data().dataLength = message->len;

        if(message->is_extended)
        {
            frameOut.Data().SetExtendedId(message->id);
        }

        for(int i = 0; i < message->len; ++i)
        {
            frameOut.Data().data[i] = message->data[i];
        }

        if (m_transfer_ros_timestamp) 
        {
            frameOut.Timestamp() = MAPSRos2Utils::ROSTimeToMAPSTimestamp(message->header.stamp);
        } 
        else 
        {
            frameOut.Timestamp() = t;
        }
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}


void MAPS_ros2_canfd_subscriber::Core()
{
    Wait4Event(isDyingEvent);
}

//De-initialization: Death() will be called once at diagram execution shutdown.
void MAPS_ros2_canfd_subscriber::Death()
{
    m_sub = nullptr;
    m_n = nullptr;
}
