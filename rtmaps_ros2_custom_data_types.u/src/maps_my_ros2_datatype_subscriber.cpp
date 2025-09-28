////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_my_ros2_datatype_subscriber.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSmy_ros2_datatype_subscriber)
    //MAPS_INPUT("iName",MAPS::FilterInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSmy_ros2_datatype_subscriber)
    MAPS_OUTPUT("id",MAPS::Integer32,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSmy_ros2_datatype_subscriber)
	MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY("transfer_ROS_timestamps",true,false,false)
    MAPS_PROPERTY("subscribe_queue_size",-1,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSmy_ros2_datatype_subscriber)
    //MAPS_ACTION("aName",MAPSmy_data_types_component::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (my_data_types_component) behaviour
MAPS_COMPONENT_DEFINITION(MAPSmy_ros2_datatype_subscriber,"my_ros2_datatype_subscriber","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions

//Initialization: Birth() will be called once at diagram execution startup.			  
void MAPSmy_ros2_datatype_subscriber::Birth()
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
    m_sub = m_n->create_subscription<my_data_type::msg::Relocalization>(topic_name.Beginning(), queue_size, std::bind(&MAPSmy_ros2_datatype_subscriber::ROSDataReceivedCallback, this, std::placeholders::_1));
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

void MAPSmy_ros2_datatype_subscriber::ROSDataReceivedCallback(const my_data_type::msg::Relocalization& message)
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


void MAPSmy_ros2_datatype_subscriber::Core()
{
    Wait4Event(isDyingEvent);
}

//De-initialization: Death() will be called once at diagram execution shutdown.
void MAPSmy_ros2_datatype_subscriber::Death()
{
    m_sub = nullptr;
    m_n = nullptr;
}
