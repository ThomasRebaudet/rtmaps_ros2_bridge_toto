#include "ros2_msgs/nav_msgs/maps_ros2_topic_odometry.h"

void MAPSros2_topic_odometry::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_output_pose_position_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_output_pose_orientation_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[1].c_str());

        m_outputsNames[2] = Name();
        m_outputsNames[2] += "_outpout_pose_covariance_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[2].c_str());

        m_outputsNames[3] = Name();
        m_outputsNames[3] += "_output_twist_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[3].c_str());

        m_outputsNames[4] = Name();
        m_outputsNames[4] += "_output_twist_covariance_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[4].c_str());
        
        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else
    {
        m_inputsNames[0] = Name();
        m_inputsNames[0] += "_input_pose_position_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[0].c_str());

        m_inputsNames[1] = Name();
        m_inputsNames[1] += "_input_pose_orientation_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[1].c_str());

        m_inputsNames[2] = Name();
        m_inputsNames[2] += "_input_pos_covariance_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[2].c_str());

        m_inputsNames[3] = Name();
        m_inputsNames[3] += "_input_twist_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[3].c_str());

        m_inputsNames[4] = Name();
        m_inputsNames[4] += "_input_twist_covariance_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[4].c_str());

        std::string s = "odom_child_frame_id_" + m_inOutName;
        NewProperty("odom_child_frame_id", s.c_str());

        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }
}

bool MAPSros2_topic_odometry::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_odometry::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<nav_msgs::msg::Odometry>(m_topicName, 10, fnc);
        Output(m_outputsNames[0].c_str()).AllocOutputBuffer(3);
        Output(m_outputsNames[1].c_str()).AllocOutputBuffer(4);
        Output(m_outputsNames[2].c_str()).AllocOutputBuffer(36);
        Output(m_outputsNames[3].c_str()).AllocOutputBuffer(6);
        Output(m_outputsNames[4].c_str()).AllocOutputBuffer(36);
    }
    else
    {
        m_pub = m_node->create_publisher<nav_msgs::msg::Odometry>(m_topicName, 100);

        for (int i=0; i < 5; i++)
        {
            m_inputs[i] = &Input(m_inputsNames[i].c_str());
        }

        std::string propertyName = "odom_child_frame_id_" + m_inOutName;
        m_child_frame_id = GetStringProperty(propertyName.c_str());
    }

    return true;
}

void MAPSros2_topic_odometry::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSTimestamp t = SynchroStartReading(5,m_inputs,m_ioelts);
	if (t < 0)
		return;

    bool error = false;
    if (m_ioelts[0]->VectorSize() != 3) 
    {
        ReportError("Input vector size on input_pose_position is not as expected. Expecting vector of 3 MAPSFloat64 (x,y,z)");
        error =true;
    }
    if(m_ioelts[1]->VectorSize() != 4) 
    {
        ReportError("Input vector size on input_pose_orientation (quaternion) is not as expected. Expecting vector of 4 MAPSFloat64 (x,y,z,w)");
        error =true;
    }
    if (m_ioelts[2]->VectorSize() != 36) 
    {
        ReportError("Input vector size on input input_pos_covariance is not as epxected. Expecting vector of 36 MAPSFloat64.");
        error =true;
    }
    if (m_ioelts[3]->VectorSize() != 6) 
    {
        ReportError("Input vector size on input input_twist is not as expected. Expecting vector of 6 MAPSFloat64 (linear x,y,z and angular x,y,z");
        error =true;
    }
    if (m_ioelts[4]->VectorSize() != 36) 
    {
        ReportError("Input vector size on input input_twist_covariance is not as expected. Expecting vector of 36 MAPSFloat64");
        error =true;
    }
    if (error) 
    {
        return;
    }

    auto msg = nav_msgs::msg::Odometry();

    if (m_child_frame_id.Len() > 0)
        msg.child_frame_id = (const char*)m_child_frame_id;

    msg.pose.pose.position.x = m_ioelts[0]->Float64();
    msg.pose.pose.position.y = m_ioelts[0]->Float64(1);
    msg.pose.pose.position.z = m_ioelts[0]->Float64(2);
    msg.pose.pose.orientation.x = m_ioelts[1]->Float64(0);
    msg.pose.pose.orientation.y = m_ioelts[1]->Float64(1);
    msg.pose.pose.orientation.z = m_ioelts[1]->Float64(2);
    msg.pose.pose.orientation.w = m_ioelts[1]->Float64(3);
    double* cov_out = msg.pose.covariance.data();
    double* cov_in =  &m_ioelts[2]->Float64();
    MAPS::Memcpy(cov_out,cov_in,36*sizeof(double));

    msg.twist.twist.linear.x = m_ioelts[3]->Float64(0);
    msg.twist.twist.linear.y = m_ioelts[3]->Float64(1);
    msg.twist.twist.linear.z = m_ioelts[3]->Float64(2);
    msg.twist.twist.angular.x = m_ioelts[3]->Float64(3);
    msg.twist.twist.angular.y = m_ioelts[3]->Float64(4);
    msg.twist.twist.angular.z = m_ioelts[3]->Float64(5);
    cov_out = msg.twist.covariance.data();
    cov_in = &m_ioelts[4]->Float64();
    MAPS::Memcpy(cov_out,cov_in,36*sizeof(double));

    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    msg.header = m_header;

    m_pub->publish(msg);
}

void MAPSros2_topic_odometry::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_odometry::TopicCallback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(odometry->header.stamp);
        }

        MAPSIOElt* ioeltout_point = StartWriting(Output(m_outputsNames[0].c_str()));
        MAPSIOElt* ioeltout_quat = StartWriting(Output(m_outputsNames[1].c_str()));
        MAPSIOElt* ioeltout_pose_cov = StartWriting(Output(m_outputsNames[2].c_str()));
        MAPSIOElt* ioeltout_twist = StartWriting(Output(m_outputsNames[3].c_str()));
        MAPSIOElt* ioeltout_twist_cov = StartWriting(Output(m_outputsNames[4].c_str()));
        ioeltout_point->Float64(0) = odometry->pose.pose.position.x;
        ioeltout_point->Float64(1) = odometry->pose.pose.position.y;
        ioeltout_point->Float64(2) = odometry->pose.pose.position.z;
        ioeltout_quat->Float64(0) = odometry->pose.pose.orientation.x;
        ioeltout_quat->Float64(1) = odometry->pose.pose.orientation.y;
        ioeltout_quat->Float64(2) = odometry->pose.pose.orientation.z;
        ioeltout_quat->Float64(3) = odometry->pose.pose.orientation.w;
        MAPSFloat64* pos_cov_out = &ioeltout_pose_cov->Float64();
        for (int i=0; i<36; i++)  
        {
            *(pos_cov_out++) = odometry->pose.covariance.at(i);
        }
        ioeltout_twist->Float64(0) = odometry->twist.twist.linear.x;
        ioeltout_twist->Float64(1) = odometry->twist.twist.linear.y;
        ioeltout_twist->Float64(2) = odometry->twist.twist.linear.z;
        ioeltout_twist->Float64(3) = odometry->twist.twist.angular.x;
        ioeltout_twist->Float64(4) = odometry->twist.twist.angular.y;
        ioeltout_twist->Float64(5) = odometry->twist.twist.angular.z;
        MAPSFloat64* twist_cov_out = &ioeltout_twist_cov->Float64();
        for (int i=0; i<36; i++) 
        {
            *(twist_cov_out++) = odometry->twist.covariance.at(i);
        }
        ioeltout_point->Timestamp()=t;
        ioeltout_quat->Timestamp()=t;
        ioeltout_pose_cov->Timestamp()=t;
        ioeltout_twist->Timestamp()=t;
        ioeltout_twist_cov->Timestamp()=t;
        StopWriting(ioeltout_point);
        StopWriting(ioeltout_quat);
        StopWriting(ioeltout_pose_cov);
        StopWriting(ioeltout_twist);
        StopWriting(ioeltout_twist_cov);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}