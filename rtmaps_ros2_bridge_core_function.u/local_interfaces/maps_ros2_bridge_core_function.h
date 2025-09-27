#ifndef __MAPS_ROS2_BRIDGE_CORE_FUNCTION_H__
#define __MAPS_ROS2_BRIDGE_CORE_FUNCTION_H__

#include "maps.h"
#include "maps_corefunction.h"
#include "maps_runshutdownlistener.h"

#include <rclcpp/rclcpp.hpp>
#include "maps_ros2_bridge_core_function_interface.h"

class MAPSROS2BridgeCoreFunction : public MAPSCoreFunction, 
			 public MAPSRunShutdownListener, public MAPSROS2BridgeCoreFunctionInterface
{
	MAPS_CF_STANDARD_HEADER_CODE()

public:
    MAPSROS2BridgeCoreFunction(const char *name);
    virtual ~MAPSROS2BridgeCoreFunction();

	//MAPSRunShutdownListener interface
	void CallbackRun();
	void CallbackShutdown();

public: // MAPSCoreFunction
	virtual void* GetInterface();

protected :
    MAPSMutex                                   m_ros2_mtx;
    MAPSEvent                                   m_evt_ros2_node_started;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    std::shared_ptr<rclcpp::Node> m_node;
    MAPSRunnable<MAPSROS2BridgeCoreFunction>    m_ros2_spin_runnable;

    std::shared_ptr<rclcpp::Node> GetROS2Node();
    void*                   SpinThread(void* instancePtr);


};

#endif
