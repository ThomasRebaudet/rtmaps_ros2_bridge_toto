#include "ros2_msgs/sensor_msgs/maps_ros2_topic_laser_scan.h"

void MAPSros2_topic_laser_scan::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_output_laser_scan_ranges_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_output_laser_scan_intensities_" + m_inOutName;
	    NewOutput("output_float64_array", m_outputsNames[1].c_str());

        m_outputsNames[2] = Name();
        m_outputsNames[2] += "_output_laser_scan_info_" + m_inOutName;
	    NewOutput("output_float64_array", m_outputsNames[2].c_str());

        std::string s  = "laser_discard_out_or_range_data_" + m_inOutName;
	    NewProperty("laser_discard_out_or_range_data", s.c_str());

        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else
    {
        m_nb_inputs = 1;
        m_inputsNames[0] = Name();
        m_inputsNames[0] += "_input_laser_scan_ranges_" + m_inOutName;
        NewInput("input_float64", m_inputsNames[0].c_str());

        std::string s  = "laser_min_angle_" + m_inOutName;
        NewProperty("laser_min_angle", s.c_str());
        s  = "laser_max_angle_" + m_inOutName;
        NewProperty("laser_max_angle", s.c_str());
        s  = "laser_angle_increment_" + m_inOutName;
        NewProperty("laser_angle_increment", s.c_str());
        s  = "laser_time_increment_" + m_inOutName;
        NewProperty("laser_time_increment", s.c_str());
        s  = "laser_scan_time_" + m_inOutName;
        NewProperty("laser_scan_time", s.c_str());
        s  = "laser_min_range_" + m_inOutName;
        NewProperty("laser_min_range", s.c_str());
        s  = "laser_max_range_" + m_inOutName;
        NewProperty("laser_max_range", s.c_str());
        s  = "laser_supports_intensities_" + m_inOutName;
        NewProperty("laser_supports_intensities", s.c_str());

        m_laser_supports_intens = GetBoolProperty(s.c_str());
        if (m_laser_supports_intens) 
        {
            m_nb_inputs = 2;
            m_inputsNames[1] = Name();
            m_inputsNames[1] += "_input_laser_scan_intensities_" + m_inOutName;
            NewInput("input_float64", m_inputsNames[1].c_str());
        }

        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }
}

bool MAPSros2_topic_laser_scan::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_laser_scan::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::LaserScan>(m_topicName, 10, fnc);

        std::string s  = "laser_discard_out_or_range_data_" + m_inOutName;
        m_discard_out_of_range = GetBoolProperty(s.c_str());
    }
    else
    {
        m_pub = m_node->create_publisher<sensor_msgs::msg::LaserScan>(m_topicName, 100);

        std::string s  = "laser_min_angle_" + m_inOutName;
        m_angle_min = GetFloatProperty(s.c_str())*MAPS_PI/180.0;
        s  = "laser_max_angle_" + m_inOutName;
        m_angle_max = GetFloatProperty(s.c_str())*MAPS_PI/180.0;
        s  = "laser_angle_increment_" + m_inOutName;
        m_angle_increment = GetFloatProperty(s.c_str())*MAPS_PI/180.0;
        s  = "laser_time_increment_" + m_inOutName;
        m_time_increment = GetFloatProperty(s.c_str());
        s  = "laser_scan_time_" + m_inOutName;
        m_scan_time = GetFloatProperty(s.c_str());
        s  = "laser_min_range_" + m_inOutName;
        m_range_min = GetFloatProperty(s.c_str());
        s  = "laser_max_range_" + m_inOutName;
        m_range_max = GetFloatProperty(s.c_str());

        if (m_nb_inputs == 2)
        {
            m_inputs[0] = &Input(m_inputsNames[0].c_str());
            m_inputs[1] = &Input(m_inputsNames[1].c_str());
        }
    }

    m_firstTime = true;

    return true;
}

void MAPSros2_topic_laser_scan::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSTimestamp t;
    if (m_nb_inputs == 1) 
	{
		m_ioelts[0] = StartReading(Input(m_inputsNames[0].c_str()));
		if (m_ioelts[0] == NULL)
			return;
		t = m_ioelts[0]->Timestamp();
	} 
	else 
	{
		t = SynchroStartReading(m_nb_inputs,m_inputs,m_ioelts);
		if (t < 0)
			return;
	}

    auto ros_laser_scan = sensor_msgs::msg::LaserScan();

    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    ros_laser_scan.header = m_header;

    ros_laser_scan.angle_min = m_angle_min;
    ros_laser_scan.angle_max = m_angle_max;
    ros_laser_scan.angle_increment = m_angle_increment;
    ros_laser_scan.time_increment = m_time_increment;
    ros_laser_scan.scan_time = m_scan_time;
    ros_laser_scan.range_min = m_range_min;
    ros_laser_scan.range_max = m_range_max;

    ros_laser_scan.ranges.resize(m_ioelts[0]->BufferSize());
    if (m_nb_inputs == 2)    
    {
        ros_laser_scan.intensities.resize(m_ioelts[1]->BufferSize());
    }

    int vectorsize_in = m_ioelts[0]->VectorSize();
    ros_laser_scan.ranges.resize(vectorsize_in);
    MAPSFloat64* data_in = &(m_ioelts[0]->Float64());
    float* data_out = ros_laser_scan.ranges.data();
    for (int i=0; i<vectorsize_in; i++) 
    {
        *(data_out++) = *(data_in++);
    }

    if (m_nb_inputs == 2)
    {
        ros_laser_scan.intensities.resize(vectorsize_in);
        MAPSFloat64* data_in_intens = &m_ioelts[1]->Float64();
        float* data_out_intens = ros_laser_scan.intensities.data();
        for (int i=0; i<vectorsize_in; i++) 
        {
            *(data_out_intens++) = *(data_in_intens++);
        }
    }

    m_pub->publish(ros_laser_scan);
}

void MAPSros2_topic_laser_scan::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_laser_scan::TopicCallback(const sensor_msgs::msg::LaserScan::SharedPtr ros_laser_scan)
{
    try 
    {
        if (m_firstTime) 
        {
            m_firstTime = false;
            m_nb_laser_scan_points = ros_laser_scan->ranges.size();
            m_nb_laser_intens_data = ros_laser_scan->intensities.size();

            Output(m_outputsNames[0].c_str()).AllocOutputBuffer(m_nb_laser_scan_points);

            if (m_nb_laser_intens_data > 0) 
            {
                Output(m_outputsNames[1].c_str()).AllocOutputBuffer(m_nb_laser_intens_data);
            }

            Output(m_outputsNames[2].c_str()).AllocOutputBuffer(7);
        }

        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(ros_laser_scan->header.stamp);
        }
        
        MAPSIOElt* out_ranges = NULL, *out_intens = NULL, *out_infos = NULL;
        out_ranges = StartWriting(Output(m_outputsNames[0].c_str()));
        if (m_nb_laser_intens_data) 
        {
            out_intens = StartWriting(Output(m_outputsNames[1].c_str()));
        }
        out_infos = StartWriting(Output(m_outputsNames[2].c_str()));
        MAPSFloat64* ranges_data = &out_ranges->Float64();

        int nb_points = ros_laser_scan->ranges.size();
        if (nb_points > m_nb_laser_scan_points) 
        {
            MAPSStreamedString ss;
            ss << "Number of scan points has increased. Problem. Truncating.";
            ReportWarning(ss);
            nb_points = m_nb_laser_scan_points;
        }

        MAPSFloat64* intens_data = NULL;
        if (m_nb_laser_intens_data)
            intens_data = &out_intens->Float64();
        int vectorsize_out = 0;
        MAPSFloat64 range;
        for (int i=0; i<nb_points; i++) 
        {
            range = ros_laser_scan->ranges[i];
            if (m_discard_out_of_range) 
            {
                if (range < ros_laser_scan->range_min || range > ros_laser_scan->range_max)
                    continue;
            }
            *(ranges_data++) = ros_laser_scan->ranges[i];
            if (m_nb_laser_intens_data) 
            {
                *(intens_data++) = ros_laser_scan->intensities[i];
            }
            vectorsize_out++;
        }
        out_infos->Float64(0) = ros_laser_scan->angle_min;
        out_infos->Float64(1) = ros_laser_scan->angle_max;
        out_infos->Float64(2) = ros_laser_scan->angle_increment;
        out_infos->Float64(3) = ros_laser_scan->time_increment;
        out_infos->Float64(4) = ros_laser_scan->scan_time;
        out_infos->Float64(5) = ros_laser_scan->range_min;
        out_infos->Float64(6) = ros_laser_scan->range_max;

        out_ranges->VectorSize() = vectorsize_out;
        if (m_nb_laser_intens_data) 
        {
            out_intens->VectorSize() = vectorsize_out;
        }

        out_infos->Timestamp() = t;
        out_ranges->Timestamp() = t;
        if (m_nb_laser_intens_data) 
        {
            out_intens->Timestamp() = t;
        }

        StopWriting(out_infos);
        if (m_nb_laser_intens_data) 
        {
            StopWriting(out_intens);
        }
        StopWriting(out_ranges);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}