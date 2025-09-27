#include "ros2_msgs/sensor_msgs/maps_ros2_topic_nav_sat_fix.h"

void MAPSros2_topic_nav_sat_fix::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_navsatfix_status_" + m_inOutName;
        NewOutput("output_int32_array", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_navsatfix_lla_pos_" + m_inOutName;
	    NewOutput("output_float64_array", m_outputsNames[1].c_str());

        m_outputsNames[2] = Name();
        m_outputsNames[2] += "_navsatfix_pos_cov_" + m_inOutName;
	    NewOutput("output_float64_array", m_outputsNames[2].c_str());

        m_outputsNames[3] = Name();
        m_outputsNames[3] += "_navsatfix_pos_cov_type_" + m_inOutName;
	    NewOutput("output_int32", m_outputsNames[3].c_str());

        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else
    {
        m_inputsNames[0] = Name();
        m_inputsNames[0] += "_navsatfix_status_" + m_inOutName;
        NewInput("input_int32", m_inputsNames[0].c_str());

        m_inputsNames[1] = Name();
        m_inputsNames[1] += "_navsatifx_pos_lla_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[1].c_str());

        m_inputsNames[2] = Name();
        m_inputsNames[2] += "_navsatfix_pos_cov_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[2].c_str());

        m_inputsNames[3] = Name();
        m_inputsNames[3] += "_navsatfix_cov_type_" + m_inOutName;
        NewInput("input_int32", m_inputsNames[3].c_str());

        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }
}

bool MAPSros2_topic_nav_sat_fix::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const sensor_msgs::msg::NavSatFix::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_nav_sat_fix::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::NavSatFix>(m_topicName, 10, fnc);
        Output(m_outputsNames[0].c_str()).AllocOutputBuffer(2);
        Output(m_outputsNames[1].c_str()).AllocOutputBuffer(3);
        Output(m_outputsNames[2].c_str()).AllocOutputBuffer(9);
    }
    else
    {
        m_pub = m_node->create_publisher<sensor_msgs::msg::NavSatFix>(m_topicName, 100);

        for (int i=0; i < 4; i++)
        {
            m_inputs[i] = &Input(m_inputsNames[i].c_str());
        }
    }

    return true;
}

void MAPSros2_topic_nav_sat_fix::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSTimestamp t;
    t = SynchroStartReading(4,m_inputs,m_ioelts);
    if (t < 0)
        return;

    if (m_ioelts[0]->VectorSize() != 2) 
    {
        MAPSStreamedString ss;
        ss << "Unexpected vector size received on input navsatfix_status (" << m_ioelts[0]->VectorSize() << "). Expecting 2: fix status and service.";
        ReportError(ss);
        return;
    }
    if (m_ioelts[1]->VectorSize() != 3) 
    {
        MAPSStreamedString ss;
        ss << "Unexpected vector size received on input navsatfix_pos_lla (" << m_ioelts[1]->VectorSize() << "). Expecting 3: latitude, longitude and altitude.";
        ReportError(ss);
        return;
    }
    if (m_ioelts[2]->VectorSize() != 9) 
    {
        MAPSStreamedString ss;
        ss << "Unexpected vector size received on input navsatfix_pos_cov" << m_ioelts[2]->VectorSize() << ".) Expecting 9.";
        ReportError(ss);
        return;
    }
    if (m_ioelts[3]->VectorSize() != 1) 
    {
        MAPSStreamedString ss;
        ss << "Unexpected vector size received on input navsatfix_cov_type (" << m_ioelts[3]->VectorSize() << "). Expecting 1.";
        ReportError(ss);
        return;
    }

    auto msg = sensor_msgs::msg::NavSatFix();
    msg.status.status = m_ioelts[0]->Integer32();
    msg.status.service = m_ioelts[0]->Integer32(1);
    
    msg.latitude = m_ioelts[1]->Float64();
    msg.longitude = m_ioelts[1]->Float64(1);
    msg.altitude = m_ioelts[1]->Float64(2);
    
    for (int i=0; i<9; i++) 
    {
        msg.position_covariance[i] = m_ioelts[2]->Float64(i);
    }
    
    msg.position_covariance_type = m_ioelts[3]->Integer32();

    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    msg.header = m_header;

    m_pub->publish(msg);
}

void MAPSros2_topic_nav_sat_fix::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_nav_sat_fix::TopicCallback(const sensor_msgs::msg::NavSatFix::SharedPtr ros_navsatfix)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(ros_navsatfix->header.stamp);
        }

        MAPSIOElt* ioeltout_status = StartWriting(Output(m_outputsNames[0].c_str()));
        MAPSIOElt* ioeltout_pos_lla = StartWriting(Output(m_outputsNames[1].c_str()));
        MAPSIOElt* ioeltout_pos_cov = StartWriting(Output(m_outputsNames[2].c_str()));
        MAPSIOElt* ioeltout_pos_cov_type = StartWriting(Output(m_outputsNames[3].c_str()));

        ioeltout_status->Integer32(0) = ros_navsatfix->status.status;
        ioeltout_status->Integer32(1) = ros_navsatfix->status.service;

        ioeltout_pos_lla->Float64(0) = ros_navsatfix->latitude;
        ioeltout_pos_lla->Float64(1) = ros_navsatfix->longitude;
        ioeltout_pos_lla->Float64(2) = ros_navsatfix->altitude;

        for (int i=0; i<9; i++) 
        {
            ioeltout_pos_cov->Float64(i) = ros_navsatfix->position_covariance[i];
        }

        ioeltout_pos_cov_type->Integer32() = ros_navsatfix->position_covariance_type;

        ioeltout_status->Timestamp() = t;
        ioeltout_pos_lla->Timestamp() = t;
        ioeltout_pos_cov->Timestamp() = t;
        ioeltout_pos_cov_type->Timestamp() = t;

        StopWriting(ioeltout_pos_cov);
        StopWriting(ioeltout_pos_cov_type);
        StopWriting(ioeltout_pos_lla);
        StopWriting(ioeltout_status);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}