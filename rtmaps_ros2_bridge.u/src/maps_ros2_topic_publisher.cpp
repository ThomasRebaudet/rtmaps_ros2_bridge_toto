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

////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

#include "maps_ros2_topic_publisher.h"	// Includes the header of this component

#include "ros2_msgs.h"

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSros2_topic_publisher)
    MAPS_INPUT("input_int32",MAPS::FilterInteger32,MAPS::FifoReader)
    MAPS_INPUT("input_int64",MAPS::FilterInteger64,MAPS::FifoReader)
    MAPS_INPUT("input_float32",MAPS::FilterFloat32,MAPS::FifoReader)
    MAPS_INPUT("input_float64",MAPS::FilterFloat64,MAPS::FifoReader)
    MAPS_INPUT("input_text",MAPS::FilterTextAscii,MAPS::FifoReader)
    MAPS_INPUT("input_stream8",MAPS::FilterStream8,MAPS::FifoReader)
    MAPS_INPUT("input_image",MAPS::FilterIplImage,MAPS::FifoReader)
    MAPS_INPUT("input_mapsimage",MAPS::FilterMAPSImage,MAPS::FifoReader)
    MAPS_INPUT("input_pointcloud_xyz",MAPS::FilterNumbers,MAPS::FifoReader)
    MAPS_INPUT("input_can_frame",MAPS::FilterCANFrame,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSros2_topic_publisher)
    //MAPS_OUTPUT("oName",MAPS::Integer,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSros2_topic_publisher)
    MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY_ENUM("topic_type","None",0,false,false)
    MAPS_PROPERTY_ENUM("message","None",0,false,false) //Text|Image|Int 32|Int 32 array|Int 64|Int 64 array|Float 32|Float 32 array|Float 64|Float 64 array|Laser scan|Twist",0,false,false)
    MAPS_PROPERTY("frame_id","map",false,false)
    MAPS_PROPERTY_ENUM("published_timestamps","RTMaps samples timestamps|ROS current time",1,false,false)
    MAPS_PROPERTY("laser_min_angle",-135.0,false,false)
    MAPS_PROPERTY("laser_max_angle",135.0,false,false)
    MAPS_PROPERTY("laser_angle_increment",1.0,false,false)
    MAPS_PROPERTY("laser_time_increment",0.0,false,false)
    MAPS_PROPERTY("laser_scan_time",0.0,false,false)
    MAPS_PROPERTY("laser_min_range",0.0,false,false)
    MAPS_PROPERTY("laser_max_range",100.0,false,false)
    MAPS_PROPERTY("laser_supports_intensities",false,false,false)
    //MAPS_PROPERTY_ENUM("pointcloud2_input_type","Distances|XYZ",0,false,false)
    MAPS_PROPERTY("pointcloud2_width",-1,false,false)
    MAPS_PROPERTY("pointcloud2_height",1,false,false)
    MAPS_PROPERTY("pointcloud2_is_dense",true,false,false)
    MAPS_PROPERTY_ENUM("pointcloud2_datatype","Auto|Integer32|Float32|Double (Float64)",0,false,false)
    MAPS_PROPERTY("odom_child_frame_id",(const char*)NULL,false,false)
    MAPS_PROPERTY("pointcloud2_nb_additional_fields", 0, false, false)
	MAPS_PROPERTY("pointcloud2_field_name",(const char*)NULL, false, false)
    MAPS_PROPERTY_ENUM("pointcloud2_field_data_type","INT8|UINT8|INT16|UINT16|INT32|UINT32|FLOAT32|FLOAT64", 0, false, false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSros2_topic_publisher)
    //MAPS_ACTION("aName",MAPSros2_topic_publisher::ActionName)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSros2_topic_publisher,"ros2_topic_publisher","1.2.0",128,
			  MAPS::Threaded,MAPS::Threaded,
			  0, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
              3, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions


MAPSros2_topic_publisher::MAPSros2_topic_publisher(const char* name, MAPSComponentDefinition& cd) :
MAPS_ros2(name,cd) {
    MAPSEnumStruct topic_types;
	for (size_t i=0; i< sizeof(s_topic_types)/sizeof(const char*); i++) {
		topic_types.enumValues->Append() = s_topic_types[i];
	}
	DirectSet(Property("topic_type"),topic_types);

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

void MAPSros2_topic_publisher::Dynamic()
{
    m_topic_type = (int)GetIntegerProperty("topic_type");
	int selected_message = (int)GetIntegerProperty("message");
	if (Property("topic_type").PropertyChanged()) {
		Property("topic_type").AcknowledgePropertyChanged();
		selected_message = 0;
	}
	MAPSEnumStruct messages;

	switch (m_topic_type) {
	case TOPIC_TYPE_STD:
		for (size_t i=0; i < sizeof(s_std_msgs)/sizeof(const char*); i++) {
			messages.enumValues->Append() = s_std_msgs[i];
		}
		break;
	case TOPIC_TYPE_SENSOR:
		for (size_t i=0; i < sizeof(s_sensor_msgs)/sizeof(const char*); i++) {
			messages.enumValues->Append() = s_sensor_msgs[i];
		}
		break;
	case TOPIC_TYPE_GEOM:
		for (size_t i=0; i < sizeof(s_geometry_msgs)/sizeof(const char*); i++) {
			messages.enumValues->Append() = s_geometry_msgs[i];
		}
		break;
    case TOPIC_TYPE_NAV:
        for (size_t i=0; i < sizeof(s_nav_msgs)/sizeof(const char*); i++) {
            messages.enumValues->Append() = s_nav_msgs[i];
        }
        break;
    case TOPIC_TYPE_CAN:
        for (size_t i=0; i < sizeof(s_can_msgs)/sizeof(const char*); i++) {
            messages.enumValues->Append() = s_can_msgs[i];
        }
        break;
	default :
		messages.enumValues->Append() = "None";
		ReportError("This topic type is not supported yet.");
		break;
	}
	if (selected_message >= messages.enumValues->Size())
		selected_message = 0;
	messages.selectedEnum = selected_message;
	DirectSet(Property("message"),messages);
	m_message = selected_message;

	m_topicInterfaces.clear();
    CreateIOsForTopics();

    for(auto& topic : m_topicInterfaces)
    {
        topic->Dynamic();
    }
}

void MAPSros2_topic_publisher::CreateIOsForTopics()
{
    std::string topicName = GetStringProperty("topic_name").Beginning();
    std::string inOutName = topicName;
    std::replace(inOutName.begin(), inOutName.end(), '/', '_');

    int message = 0;
    switch (m_topic_type) {
	case TOPIC_TYPE_STD:
		message = m_message + TOPIC_TYPE_STD_OFFSET;
		break;
	case TOPIC_TYPE_SENSOR:
		message = m_message + TOPIC_TYPE_SENSOR_OFFSET;
		break;
	case TOPIC_TYPE_GEOM:
		message = m_message + TOPIC_TYPE_GEOM_OFFSET;
		break;
    case TOPIC_TYPE_NAV:
        message = m_message + TOPIC_TYPE_NAV_OFFSET;
        break;
    case TOPIC_TYPE_CAN:
        message = m_message + TOPIC_TYPE_CAN_OFFSET;
        break;
	default :
		ReportError("This topic type is not supported yet.");
		break;
	}

	switch(message) {
	case STD_MSG_INT32:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_int32>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case STD_MSG_INT32_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_int32_array>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case STD_MSG_INT64:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_int64>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case STD_MSG_INT64_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_int64_array>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case STD_MSG_FLOAT32:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_float32>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case STD_MSG_FLOAT32_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_float32_array>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case STD_MSG_FLOAT64:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_float64>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case STD_MSG_FLOAT64_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_float64_array>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case STD_MSG_TEXT:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_string>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
    case STD_MSG_UINT8_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_uint8_array>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
    case SENSOR_MSG_COMPRESSED_IMAGE:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_compressed_image>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case SENSOR_MSG_IMAGE:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_image>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
	case SENSOR_MSG_LASER_SCAN:
	    m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_laser_scan>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
    case SENSOR_MSG_POINT_CLOUD2:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_pointcloud2>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
        break;
    case SENSOR_MSG_NAV_SAT_FIX:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_nav_sat_fix>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
        break;
    case SENSOR_MSG_IMU:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_imu>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
        break;
    case GEOM_MSG_POINT:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_point>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
		break;
    case GEOM_MSG_TWIST:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_twist>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
        break;
    case GEOM_MSG_TWIST_STAMPED:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_twist_stamped>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
        break;
    case NAV_MSG_ODOMETRY:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_odometry>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
        break;
    case CAN_MSG_FRAME:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_frame>(this, m_n, Endpoint_Type::Publisher, topicName, inOutName));
        break;
    default:
		ReportError("This topic is not supported yet.");
	}
}

void MAPSros2_topic_publisher::Birth()
{
    try 
    {
        for(auto& topic : m_topicInterfaces)
        {
            topic->Birth();
        }
    }
    catch (std::exception& e)
    {
        MAPSStreamedString ss;
        ss << "Could not create the ROS2 publisher: " << e.what();
        Error(ss);
    }
}

void MAPSros2_topic_publisher::Core()
{
	for(auto& topic : m_topicInterfaces)
    {
        topic->Core();
    }
}

void MAPSros2_topic_publisher::Death()
{
    for(auto& topic : m_topicInterfaces)
    {
        topic->Death();
    }
}
