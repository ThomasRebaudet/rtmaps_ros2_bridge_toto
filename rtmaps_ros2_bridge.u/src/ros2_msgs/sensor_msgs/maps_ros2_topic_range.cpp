#include "ros2_msgs/sensor_msgs/maps_ros2_topic_range.h"

void MAPSros2_topic_range::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_range_" + m_inOutName;
        NewOutput("output_float32", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_range_info_" + m_inOutName;
	    NewOutput("output_float64_array", m_outputsNames[1].c_str());

        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else//not available
    {

    }
}

bool MAPSros2_topic_range::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const sensor_msgs::msg::Range::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_range::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::Range>(m_topicName, 10, fnc);
        Output(m_outputsNames[1].c_str()).AllocOutputBuffer(4);
    }
    else//not available
    {

    }

    return true;
}

void MAPSros2_topic_range::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;
}

void MAPSros2_topic_range::Death()
{
    m_sub.reset();
}

void MAPSros2_topic_range::TopicCallback(const sensor_msgs::msg::Range::SharedPtr ros_range)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(ros_range->header.stamp);
        }

        MAPSIOElt* ioeltout = StartWriting(Output(m_outputsNames[0].c_str()));
        ioeltout->Float32() = ros_range->range;
        ioeltout->Timestamp() = t;
        StopWriting(ioeltout);

        ioeltout = StartWriting(Output(m_outputsNames[1].c_str()));
        ioeltout->Float64(0) = ros_range->radiation_type;
        ioeltout->Float64(1) = ros_range->field_of_view;
        ioeltout->Float64(2) = ros_range->min_range;
        ioeltout->Float64(3) = ros_range->max_range;
        ioeltout->Timestamp() = t;
        StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}