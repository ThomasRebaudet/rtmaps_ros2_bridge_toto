#include "ros2_msgs/sensor_msgs/maps_ros2_topic_pointcloud.h"

void MAPSros2_topic_pointcloud::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_output_point_cloud_" + m_inOutName;
        NewOutput("output_float32_array", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_output_point_cloud_channel_sizes_" + m_inOutName;
	    NewOutput("output_int32_array", m_outputsNames[1].c_str());

        m_outputsNames[2] = Name();
        m_outputsNames[2] += "_output_point_cloud_channels_" + m_inOutName;
	    NewOutput("output_float32_array", m_outputsNames[2].c_str());

        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else//not available
    {

    }
}

bool MAPSros2_topic_pointcloud::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const sensor_msgs::msg::PointCloud::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_pointcloud::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::PointCloud>(m_topicName, 10, fnc);

        m_firstTime = true;
    }
    else//not available
    {
    }

    return true;
}

void MAPSros2_topic_pointcloud::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;
}

void MAPSros2_topic_pointcloud::Death()
{
    m_sub.reset();
}

void MAPSros2_topic_pointcloud::TopicCallback(const sensor_msgs::msg::PointCloud::SharedPtr ros_point_cloud)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(ros_point_cloud->header.stamp);
        }

        if (m_firstTime) 
        {
            m_firstTime = false;
            m_nb_points = ros_point_cloud->points.size();
            m_nb_channels = ros_point_cloud->channels.size();

            Output(m_outputsNames[0].c_str()).AllocOutputBuffer(m_nb_points*3);
            if (m_nb_channels > 0) 
            {
                Output(m_outputsNames[1].c_str()).AllocOutputBuffer(m_nb_channels);
                Output(m_outputsNames[2].c_str()).AllocOutputBuffer(m_nb_points);
            }
        }

        MAPSIOElt* out_points_xyz = nullptr, *out_nb_channels = nullptr, *out_distances = nullptr;
        out_points_xyz = StartWriting(Output(m_outputsNames[0].c_str()));
        if (m_nb_channels) 
        {
            out_nb_channels = StartWriting(Output(m_outputsNames[1].c_str()));
            out_distances = StartWriting(Output(m_outputsNames[2].c_str()));
        }
        MAPSFloat32* xyz_points = &out_points_xyz->Float32();

        int nb_points = ros_point_cloud->points.size();
        if (nb_points > m_nb_points) 
        {
            MAPSStreamedString ss;
            ss << "Number of scan points has increased. Problem. Truncating.";
            ReportWarning(ss);
            nb_points = m_nb_points;
        }

        for (int i=0; i<nb_points; i++) 
        {
            *(xyz_points++) = ros_point_cloud->points[i].x;
            *(xyz_points++) = ros_point_cloud->points[i].y;
            *(xyz_points++) = ros_point_cloud->points[i].z;
        }
        out_points_xyz->VectorSize() = nb_points*3;

        MAPSFloat32* channels_data;
        MAPSInt32* channels_sizes;
        if (m_nb_channels) 
        {
            channels_sizes = &out_nb_channels->Integer32();
            channels_data = &out_distances->Float32();
        }
        for (int i=0; i<m_nb_channels; i++) 
        {
            int nb_ranges_on_channel = ros_point_cloud->channels[i].values.size();
            *(channels_sizes++) = nb_ranges_on_channel;
            for (int j=0; j<nb_ranges_on_channel; j++) 
            {
                *(channels_data++) = ros_point_cloud->channels[i].values[j];
            }
        }

        if (m_nb_channels) 
        {
            out_nb_channels->VectorSize() = m_nb_channels;
            out_distances->VectorSize() = nb_points;
            out_nb_channels->Timestamp() = t;
            out_distances->Timestamp() = t;
            StopWriting(out_nb_channels);
            StopWriting(out_distances);
        }
        out_points_xyz->Timestamp() = t;
        StopWriting(out_points_xyz);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}