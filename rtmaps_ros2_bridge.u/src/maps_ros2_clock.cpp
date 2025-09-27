/*
 * Copyright (c) 2020 Intempora
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Intempora nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL INTEMPORA
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "maps_ros2_clock.h"	// Includes the header of this component
#include "maps_corefunction.h"


// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSros2_clock)
    //MAPS_INPUT("iName",MAPS::FilterInteger,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSros2_clock)
    MAPS_OUTPUT("ros_time",MAPS::Integer64,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSros2_clock)
    MAPS_PROPERTY("use_sim_time",false,false,true)
    MAPS_PROPERTY("max_timespeed",100,false,true)
	MAPS_PROPERTY_SUBTYPE("ros_time_publication_period",-1,false,true,MAPS::PropertySubTypeTime)
	MAPS_PROPERTY_SUBTYPE("ros_use_sim_time_init_timeout",10000000,false,false,MAPS::PropertySubTypeTime)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSros2_clock)
    //MAPS_ACTION("aName",MAPSros2_clock::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ros_clock) behaviour
MAPS_COMPONENT_DEFINITION(MAPSros2_clock,"ros2_clock","1.0.1",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions


void MAPSros2_clock::InitClock()
{
}

void MAPSros2_clock::RunClock()
{
    m_node = MAPSRos2Utils::GetROS2Node();
	if (!m_node)
	{
		m_clockDisabled = true;
		ReportError("Could not init ROS.");
		return;
	}

	m_clockHasShutdown=false;

    SetAbsoluteTimeSpeed((int)GetIntegerProperty("max_timespeed")*10);
	if (MAPS::GetFirstTimestamp() != 0 || MAPS::GetLastTimestamp() != 0) //We've got a Player in the diagram...
	{
		m_clockDisabled = true;
		ReportError("Trying to synchronize the RTMaps clock with the GPS UTC time is not allowed in replay mode.");
	}

    try
    {
        bool useSimTime = GetBoolProperty("use_sim_time");
        rclcpp::Parameter simTimeParm("use_sim_time", useSimTime);
        rcl_interfaces::msg::SetParametersResult result = m_node->set_parameter(simTimeParm);

        if(result.successful && useSimTime)
        {
            MAPSTimestamp ros_time = 0;
            MAPSTimestamp current_time = 0;

            bool is_ok = false;
            int n_tries = GetIntegerProperty("ros_use_sim_time_init_timeout") / 100000;
            ReportInfo("Initializing clock against ROS simulation time. Waiting for ROS simulation time validity...");
            for (int count = 0; count < n_tries; count++)
            {
                ros_time = MAPSRos2Utils::ROSTimeToMAPSTimestamp(m_node->now());

                if(ros_time <= 0)
                {
                    MAPS::Sleep(100000);
                    continue;
                }

                MAPSAbsoluteTime ros_time_at;
                MAPS::Timestamp2AbsoluteTimeUTC(ros_time, &ros_time_at);
                MAPSAbsoluteTime at;
                MAPS::GetAbsoluteTimeUTC(&at);
                current_time = MAPS::AbsoluteTimeUTC2Timestamp(&at);

                if (abs(ros_time - current_time) > 2000000)
                {
                    is_ok = true;
                    break;
                }
                MAPS::Sleep(100000);
            }

            if (!is_ok)
            {
                ReportError("It seems we should be using the ROS simulation time (ROS param use_sim_time is true)\n\
                but ROS keeps on reporting the wall time. Make sure the rosbag play is started with argument --clock.\n\
                Otherswise it would prevent the RTMaps recorder from working in case\n\
                we get ROS data samples with a timestamp that is far from the wall time (e.g. if their timestamps\n\
                correspond to the simulation time), and also if later the ROS time jumps backwards to simulation time.");

                MAPS::AsynchParse("shutdown");
            } 
            else 
            {
                ReportInfo("ROS simulation time initialization ok.");
            }
        }
    }
    catch(const std::exception& e)
    {
        ReportError(e.what());
    }
}

MAPSTimestamp MAPSros2_clock::CurrentTime()
{
	if (true == m_clockDisabled || true == m_clockHasShutdown)
		return 0;
	m_MyTimeMonitor.Lock();
    MAPSTimestamp t = MAPSRos2Utils::ROSTimeToMAPSTimestamp(m_node->now());
	m_MyTimeMonitor.Release();
	return t;
}

void MAPSros2_clock::Birth()
{
}

void MAPSros2_clock::Core() 
{
	MAPSInt64 t = GetIntegerProperty("ros_time_publication_period");
	if(t!=-1)
		Rest(t);
	else
		Wait4Event(isDyingEvent);

	MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Integer64() = MAPSRos2Utils::ROSTimeToMAPSTimestamp(m_node->now());
	StopWriting(ioeltout);
}

void MAPSros2_clock::Death()
{
}

void MAPSros2_clock::ShutdownClock()
{
    m_clockHasShutdown = true;
	m_clockDisabled = false;
 }
