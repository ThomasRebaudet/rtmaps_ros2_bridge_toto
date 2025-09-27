#include "ros2_msgs/sensor_msgs/maps_ros2_topic_imu.h"

void MAPSros2_topic_imu::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_orientation_quaternion_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_angular_velocities_" + m_inOutName;
	    NewOutput("output_float64_array", m_outputsNames[1].c_str());

        m_outputsNames[2] = Name();
        m_outputsNames[2] += "_accelerations_" + m_inOutName;
	    NewOutput("output_float64_array", m_outputsNames[2].c_str());

        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else
    {
        m_inputsNames[0] = Name();
        m_inputsNames[0] += "_orientation_quaternion_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[0].c_str());

        m_inputsNames[1] = Name();
        m_inputsNames[1] += "_angular_velocities_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[1].c_str());

        m_inputsNames[2] = Name();
        m_inputsNames[2] += "_accelerations_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[2].c_str());

        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }
}

bool MAPSros2_topic_imu::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const sensor_msgs::msg::Imu::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_imu::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::Imu>(m_topicName, 10, fnc);
        Output(m_outputsNames[0].c_str()).AllocOutputBuffer(4);
        Output(m_outputsNames[1].c_str()).AllocOutputBuffer(3);
        Output(m_outputsNames[2].c_str()).AllocOutputBuffer(3);
    }
    else
    {
        m_pub = m_node->create_publisher<sensor_msgs::msg::Imu>(m_topicName, 100);

        for (int i=0; i < 3; i++)
        {
            m_inputs[i] = &Input(m_inputsNames[i].c_str());
        }
    }

    return true;
}

void MAPSros2_topic_imu::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSTimestamp t;
    t = SynchroStartReading(3,m_inputs,m_ioelts);
    if (t < 0)
        return;

    if (m_ioelts[0]->VectorSize() != 4) 
    {
        MAPSStreamedString ss;
        ss << "Unexpected vector size received on input orientation_quaternion (" << m_ioelts[1]->VectorSize() << "). Expecting 4.";
        ReportError(ss);
        return;
    }
    if (m_ioelts[1]->VectorSize() != 3) 
    {
        MAPSStreamedString ss;
        ss << "Unexpected vector size received on input angular_velocities (" << m_ioelts[1]->VectorSize() << "). Expecting 3.";
        ReportError(ss);
        return;
    }
    if (m_ioelts[2]->VectorSize() != 3) 
    {
        MAPSStreamedString ss;
        ss << "Unexpected vector size received on input accelerations" << m_ioelts[2]->VectorSize() << ".) Expecting 3.";
        ReportError(ss);
        return;
    }

    auto msg = sensor_msgs::msg::Imu();
    msg.orientation.x =         m_ioelts[0]->Float64();
    msg.orientation.y =         m_ioelts[0]->Float64(1);
    msg.orientation.z =         m_ioelts[0]->Float64(2);
    msg.orientation.w =         m_ioelts[0]->Float64(3);
    msg.angular_velocity.x =    m_ioelts[1]->Float64();
    msg.angular_velocity.y =    m_ioelts[1]->Float64(1);
    msg.angular_velocity.z =    m_ioelts[1]->Float64(2);
    msg.linear_acceleration.x = m_ioelts[2]->Float64();
    msg.linear_acceleration.y = m_ioelts[2]->Float64(1);
    msg.linear_acceleration.z = m_ioelts[2]->Float64(2);

    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    msg.header = m_header;

    m_pub->publish(msg);
}

void MAPSros2_topic_imu::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_imu::TopicCallback(const sensor_msgs::msg::Imu::SharedPtr ros_imu)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(ros_imu->header.stamp);
        }

        MAPSIOElt* out_orientation, *out_gyro, *out_acc;
        out_orientation = StartWriting(Output(m_outputsNames[0].c_str()));
        out_gyro = StartWriting(Output(m_outputsNames[1].c_str()));
        out_acc = StartWriting(Output(m_outputsNames[2].c_str()));
        out_orientation->Float64(0) = ros_imu->orientation.x;
        out_orientation->Float64(1) = ros_imu->orientation.y;
        out_orientation->Float64(2) = ros_imu->orientation.z;
        out_orientation->Float64(3) = ros_imu->orientation.w;
        
        out_gyro->Float64(0) = ros_imu->angular_velocity.x;
        out_gyro->Float64(1) = ros_imu->angular_velocity.y;
        out_gyro->Float64(2) = ros_imu->angular_velocity.z;
        
        out_acc->Float64(0) = ros_imu->linear_acceleration.x;
        out_acc->Float64(1) = ros_imu->linear_acceleration.y;
        out_acc->Float64(2) = ros_imu->linear_acceleration.z;
        
        out_orientation->Timestamp() = t;
        out_gyro->Timestamp() = t;
        out_acc->Timestamp() = t;
        StopWriting(out_acc);
        StopWriting(out_gyro);
        StopWriting(out_orientation);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}