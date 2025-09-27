#include "maps_ros2_bridge_core_function.h"


MAPS_CF_DEFINITION(MAPSROS2BridgeCoreFunction,MAPSCoreFunction,ros2_bridge_core_function,"1.0.0")

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSROS2BridgeCoreFunction)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSROS2BridgeCoreFunction)
MAPS_END_ACTIONS_DEFINITION

MAPSCoreFunction* MAPSROS2BridgeCoreFunction::InstFunct(const char *name,MAPSCFDefinition* kf)
{
    if (kf->refCounter==0) {
        ++kf->refCounter;
        return new MAPSROS2BridgeCoreFunction(name);
    }
    else {
        MAPS::ReportError("The ROS bridge kernel function has already been loaded. Cannot load another one.",0);
        return NULL;
    }
}

MAPSROS2BridgeCoreFunction::MAPSROS2BridgeCoreFunction(const char* name)
: MAPSCoreFunction(name)
{
    MAPSProperty* pid_prop = MAPS::Property("Engine.pid");
    int current_pid = 0;
    if (pid_prop != nullptr)
    {
        current_pid = pid_prop->IntegerValue();
    }

    std::stringstream process_name;
    process_name << "rtmaps_ros2_bridge_" << current_pid;
    std::string process_name_str = process_name.str();

    const char* argv[2] = { process_name_str.c_str(), nullptr };
    rclcpp::init(1, argv, rclcpp::InitOptions());

    std::stringstream node_name;
    node_name << "rtmaps_ros2_node_" << current_pid;
    std::string node_name_str = node_name.str();
    m_node = rclcpp::Node::make_shared(node_name_str.c_str());
    m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_node(m_node);

    m_ros2_spin_runnable.Init(&MAPSROS2BridgeCoreFunction::SpinThread, this);
}

MAPSROS2BridgeCoreFunction::~MAPSROS2BridgeCoreFunction()
{
    try 
    {
        rclcpp::shutdown();
    }
    catch (std::exception& e)
    {
        ReportError(e.what());
    }
}

std::shared_ptr<rclcpp::Node> MAPSROS2BridgeCoreFunction::GetROS2Node()
{
    return m_node;
}

/*****************************************************************************
** MAPSCoreFunction implementation
*****************************************************************************/

void* MAPSROS2BridgeCoreFunction::GetInterface()
{
	return static_cast<MAPSROS2BridgeCoreFunctionInterface*>(this);
}


void MAPSROS2BridgeCoreFunction::CallbackRun()
{
    m_ros2_spin_runnable.Start();
}

void MAPSROS2BridgeCoreFunction::CallbackShutdown()
{
    m_executor->cancel();
    m_ros2_spin_runnable.Stop();
}


void* MAPSROS2BridgeCoreFunction::SpinThread(void* )
{
    try 
    {
        m_executor->spin();
    }
    catch (std::exception& e)
    {
        MAPS::ReportError(e.what());
        return nullptr;
    }

    return this;
}
