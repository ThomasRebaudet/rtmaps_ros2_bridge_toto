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

#include "maps_ros2_topic_subscriber.h"

MAPS_BEGIN_INPUTS_DEFINITION(MAPSros2_topic_subscriber)
    //MAPS_INPUT("iName",MAPS::FilterInteger,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSros2_topic_subscriber)
    MAPS_OUTPUT("output_text",MAPS::TextAscii,NULL,NULL,0)
    MAPS_OUTPUT_FIFOSIZE("output_image",MAPS::IplImage,NULL,NULL,0,10)
    MAPS_OUTPUT_FIFOSIZE("output_compressed_image",MAPS::MAPSImage,NULL,NULL,0,10)
    MAPS_OUTPUT("output_int32",MAPS::Integer32,NULL,NULL,1)
    MAPS_OUTPUT("output_int32_array",MAPS::Integer32,NULL,NULL,0)
    MAPS_OUTPUT("output_int64",MAPS::Integer64,NULL,NULL,1)
    MAPS_OUTPUT("output_int64_array",MAPS::Integer64,NULL,NULL,0)
    MAPS_OUTPUT("output_float32",MAPS::Float32,NULL,NULL,1)
    MAPS_OUTPUT("output_float32_array",MAPS::Float32,NULL,NULL,0)
    MAPS_OUTPUT("output_float64",MAPS::Float64,NULL,NULL,1)
    MAPS_OUTPUT("output_float64_array",MAPS::Float64,NULL,NULL,0)
    MAPS_OUTPUT("output_uint8_array",MAPS::Stream8,NULL,NULL,0)
	MAPS_OUTPUT_USER_STRUCTURE("array_layout",ROSArrayLayout)
	//LASER SCAN
	MAPS_OUTPUT("output_laser_scan_ranges",MAPS::Float64,NULL,NULL,0)
	MAPS_OUTPUT("output_laser_scan_intensities",MAPS::Float64,NULL,NULL,0)
	MAPS_OUTPUT("output_laser_scan_info",MAPS::Float64,NULL,NULL,7)
	//POINT CLOUD
	MAPS_OUTPUT("output_point_cloud",MAPS::Float32,NULL,NULL,0)
	MAPS_OUTPUT("output_point_cloud_channel_sizes",MAPS::Integer32,NULL,NULL,0)
	MAPS_OUTPUT("output_point_cloud_channels",MAPS::Float32,NULL,NULL,0)
	//POINT CLOUD 2
	MAPS_OUTPUT("output_point_cloud_2_info",MAPS::Integer32,NULL,NULL,6) //width, height, is_big_endian, point_step, row_step, is_dense
	MAPS_OUTPUT("output_point_cloud_2_fields_names", MAPS::TextAscii,NULL,NULL,0) // fields names, separated by '|'
	MAPS_OUTPUT("output_point_cloud_2_fields_info", MAPS::Integer32,NULL,NULL,0) // vector of (offset, datatype, count)
	MAPS_OUTPUT("output_point_cloud_2_data",MAPS::Stream8,NULL,NULL,0) //data
	//JOYSTICK
	MAPS_OUTPUT("output_joy_axes",MAPS::Float32,NULL,NULL,0)
	MAPS_OUTPUT("output_joy_buttons",MAPS::Integer32,NULL,NULL,0)
	//TWIST
	MAPS_OUTPUT("output_twist",MAPS::Float64,NULL,NULL,6)
    //COVARIANCE
    MAPS_OUTPUT("output_covariance",MAPS::Float64,NULL,NULL,36)
	//IMU MESSAGES
	MAPS_OUTPUT("orientation_quaternion",MAPS::Float64,NULL,NULL,4)
	MAPS_OUTPUT("angular_velocities",MAPS::Float64,NULL,NULL,3)
	MAPS_OUTPUT("accelerations",MAPS::Float64,NULL,NULL,3)
	//RANGE MESSAGES
	MAPS_OUTPUT("range_info",MAPS::Float64,NULL,NULL,4)
    //NAVSATFIX MESSAGES
    MAPS_OUTPUT("navsatfix_status",MAPS::Integer32,NULL,NULL,2)
    MAPS_OUTPUT("navsatfix_lla_pos",MAPS::Float64,NULL,NULL,3)
    MAPS_OUTPUT("navsatfix_pos_cov",MAPS::Float64,NULL,NULL,9)
    MAPS_OUTPUT("navsatfix_pos_cov_type",MAPS::Integer32,NULL,NULL,1)
    //VISUALIZATION MARKERS
    MAPS_OUTPUT("output_marker_arrow", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_cube", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_sphere", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_cylinder", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_line_strip", MAPS::Float64, NULL,NULL, 0)
    MAPS_OUTPUT("output_marker_line_list", MAPS::Float64,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_cube_list", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_sphere_list", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_points", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_text_view_facing", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_mesh_resource", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_triangle_list", MAPS::Float64,NULL,NULL,0)
    //CAN
    MAPS_OUTPUT("output_can_frame", MAPS::CANFrame,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSros2_topic_subscriber)
	MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
	MAPS_PROPERTY_ENUM("topic_type","None",0,false,false)
	MAPS_PROPERTY_ENUM("message","None",0,false,false) //Text|Image|Int 32|Int 32 array|Int 64|Int 64 array|Float 32|Float 32 array|Float 64|Float 64 array|Laser scan|Twist",0,false,false)
    MAPS_PROPERTY("subscribe_queue_size",-1,false,false)
    MAPS_PROPERTY("transfer_ROS_timestamps",true,false,false)
    MAPS_PROPERTY("max_text_length",255,false,false)
    MAPS_PROPERTY("max_array_size",256,false,false)
    MAPS_PROPERTY("laser_discard_out_or_range_data",false,false,false)
    MAPS_PROPERTY("max_nb_points",-1,false,false)
    MAPS_PROPERTY("compressed_image_width",640,false,false)
    MAPS_PROPERTY("compressed_image_height",480,false,false)
    MAPS_PROPERTY("pointcloud2_nb_additional_fields", 0, false, false)
	MAPS_PROPERTY("pointcloud2_field_name",(const char*)NULL, false, false)
	MAPS_PROPERTY_ENUM("pointcloud2_field_data_type","INT8|UINT8|INT16|UINT16|INT32|UINT32|FLOAT32|FLOAT64", 0, false, false)
MAPS_END_PROPERTIES_DEFINITION



MAPS_BEGIN_ACTIONS_DEFINITION(MAPSros2_topic_subscriber)
    //MAPS_ACTION("aName",MAPSros2_topic_subscriber::ActionName)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSros2_topic_subscriber,"ros2_topic_subscriber","1.2.0",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  0, // Nb of outputs. Leave -1 to use the number of declared output definitions
              4, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions

MAPSros2_topic_subscriber::MAPSros2_topic_subscriber(const char* name, MAPSComponentDefinition& cd) :
MAPS_ros2(name,cd)
{
	MAPSEnumStruct topic_types;
    for (unsigned int i=0; i< sizeof(s_topic_types)/sizeof(const char*); i++) 
    {
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

void MAPSros2_topic_subscriber::Dynamic()
{
	m_topic_type = (int)GetIntegerProperty("topic_type");
	int selected_message = (int)GetIntegerProperty("message");
    bool use_default_message_idx = false;
	if (Property("topic_type").PropertyChanged()) 
    {
		Property("topic_type").AcknowledgePropertyChanged();
        use_default_message_idx = true;
	}
	MAPSEnumStruct messages;

	switch (m_topic_type) 
    {
	case TOPIC_TYPE_STD:
        for (unsigned int i=0; i < sizeof(s_std_msgs)/sizeof(const char*); i++) 
        {
			messages.enumValues->Append() = s_std_msgs[i];
		}
        if (use_default_message_idx)
            selected_message = 0;
		break;
	case TOPIC_TYPE_SENSOR:
        for (unsigned int i=0; i < sizeof(s_sensor_msgs)/sizeof(const char*); i++) 
        {
			messages.enumValues->Append() = s_sensor_msgs[i];
		}
        if (use_default_message_idx)
            selected_message = SENSOR_MSG_POINT_CLOUD2;
        break;
	case TOPIC_TYPE_GEOM:
        for (unsigned int i=0; i < sizeof(s_geometry_msgs)/sizeof(const char*); i++) 
        {
			messages.enumValues->Append() = s_geometry_msgs[i];
		}
        if (use_default_message_idx)
            selected_message = 0;
        break;
    case TOPIC_TYPE_NAV:
        for (unsigned int i=0; i < sizeof(s_nav_msgs)/sizeof(const char*); i++) 
        {
            messages.enumValues->Append() = s_nav_msgs[i];
        }
        if (use_default_message_idx)
            selected_message = 0;
        break;
    case TOPIC_TYPE_VISU:
        for (unsigned int i=0; i < sizeof(s_visu_msgs)/sizeof(const char*); i++) 
        {
            messages.enumValues->Append() = s_visu_msgs[i];
        }
        if (use_default_message_idx)
            selected_message = 0;
        break;
    case TOPIC_TYPE_CAN:
        for (unsigned int i=0; i < sizeof(s_can_msgs)/sizeof(const char*); i++) 
        {
            messages.enumValues->Append() = s_can_msgs[i];
        }
        if (use_default_message_idx)
            selected_message = 0;
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

void MAPSros2_topic_subscriber::CreateIOsForTopics()
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
    case TOPIC_TYPE_VISU:
        message = m_message + TOPIC_TYPE_VISU_OFFSET;
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
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_int32>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case STD_MSG_INT32_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_int32_array>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case STD_MSG_INT64:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_int64>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case STD_MSG_INT64_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_int64_array>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case STD_MSG_FLOAT32:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_float32>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case STD_MSG_FLOAT32_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_float32_array>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case STD_MSG_FLOAT64:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_float64>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case STD_MSG_FLOAT64_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_float64_array>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case STD_MSG_TEXT:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_string>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
    case STD_MSG_UINT8_ARRAY:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_uint8_array>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
    case SENSOR_MSG_IMAGE :
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_image>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
    case SENSOR_MSG_COMPRESSED_IMAGE:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_compressed_image>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
        break;
	case SENSOR_MSG_LASER_SCAN :
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_laser_scan>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case SENSOR_MSG_JOY :
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_joy>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case SENSOR_MSG_IMU :
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_imu>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case SENSOR_MSG_POINT_CLOUD:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_pointcloud>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case SENSOR_MSG_POINT_CLOUD2:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_pointcloud2>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case SENSOR_MSG_RANGE:
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_range>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
    case SENSOR_MSG_NAV_SAT_FIX:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_nav_sat_fix>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
        break;
    case GEOM_MSG_POINT :
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_point>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case GEOM_MSG_POSE :
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_pose>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case GEOM_MSG_POSE_STAMPED :
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_pose_stamped>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
	case GEOM_MSG_TWIST :
		m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_twist>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
		break;
    case GEOM_MSG_TWIST_STAMPED:
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_twist_stamped>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
        break;
    case NAV_MSG_ODOMETRY :
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_odometry>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
        break;
     case VISU_MSG_MARKER :
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_marker>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
        break;
    case VISU_MSG_MARKER_ARRAY :
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_marker_array>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
        break;
    case CAN_MSG_FRAME :
        m_topicInterfaces.push_back(std::make_unique<MAPSros2_topic_frame>(this, m_n, Endpoint_Type::Subscriber, topicName, inOutName));
        break;
    default:
		ReportError("This topic is not supported yet.");
	}
}

void MAPSros2_topic_subscriber::Birth()
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

void MAPSros2_topic_subscriber::Core()
{
    Wait4Event(isDyingEvent);
}

void MAPSros2_topic_subscriber::Death()
{
    for(auto& topic : m_topicInterfaces)
    {
        topic->Death();
    }
}
