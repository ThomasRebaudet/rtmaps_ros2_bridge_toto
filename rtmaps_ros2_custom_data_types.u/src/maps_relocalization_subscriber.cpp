////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_relocalization_subscriber.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSrelocalization_subscriber)
    //MAPS_INPUT("iName",MAPS::FilterInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSrelocalization_subscriber)
    MAPS_OUTPUT("state", MAPS::UnsignedInteger8, NULL, NULL, 1)          // uint8 scalar
    MAPS_OUTPUT("state_human_readable", MAPS::TextUTF8, NULL, NULL, 1)  // string scalar
    MAPS_OUTPUT("pose", MAPS::Float64, NULL, NULL, 16)                  // float64[16] vector
    MAPS_OUTPUT("pose_cov", MAPS::Float64, NULL, NULL, 36)              // float64[36] vector
    MAPS_OUTPUT("pose_std", MAPS::Float32, NULL, NULL, 6)               // float32[6] vector
    MAPS_OUTPUT("lla", MAPS::Float32, NULL, NULL, 3)                    // float32[3] vector
    MAPS_OUTPUT("velocity", MAPS::Float32, NULL, NULL, 6)               // float32[6] vector
    MAPS_OUTPUT("twist", MAPS::Float32, NULL, NULL, 6)                  // float32[6] vector
    MAPS_OUTPUT("overlap", MAPS::Float32, NULL, NULL, 1)                // float32 scalar
MAPS_END_OUTPUTS_DEFINITION


// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSrelocalization_subscriber)
	MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY("transfer_ROS_timestamps",true,false,false)
    MAPS_PROPERTY("subscribe_queue_size",-1,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSrelocalization_subscriber)
    //MAPS_ACTION("aName",MAPSmy_data_types_component::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (my_data_types_component) behaviour
MAPS_COMPONENT_DEFINITION(MAPSrelocalization_subscriber,"relocalization_subscriber","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions

//Initialization: Birth() will be called once at diagram execution startup.			  
void MAPSrelocalization_subscriber::Birth()
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
    queue_size = queue_size == -1?1000:queue_size;
    m_sub = m_n->create_subscription<my_data_type::msg::Relocalization>(topic_name.Beginning(), queue_size, std::bind(&MAPSrelocalization_subscriber::ROSDataReceivedCallback, this, std::placeholders::_1));
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

void MAPSrelocalization_subscriber::ROSDataReceivedCallback(const my_data_type::msg::Relocalization& message)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();

        MAPSIOElt* ioeltout = StartWriting(Output(0));
        ioeltout->Integer32() = message.state;
        if (m_transfer_ros_timestamp) 
        {
            ioeltout->Timestamp() = MAPSRos2Utils::ROSTimeToMAPSTimestamp(message.header.stamp);
        } 
        else 
        {
            ioeltout->Timestamp() = t;
        }
        StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}


void MAPSrelocalization_subscriber::Core()
{
    Wait4Event(isDyingEvent);
}
//De-initialization: Death() will be called once at diagram execution shutdown.
void MAPSrelocalization_subscriber::Death()
{
    m_sub = nullptr;
    m_n = nullptr;
}
