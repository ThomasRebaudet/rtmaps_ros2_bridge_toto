#include "ros2_msgs/visualization_msgs/maps_ros2_topic_marker.h"

void MAPSros2_topic_marker::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_output_marker_arrow_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_output_marker_cube_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[1].c_str());

        m_outputsNames[2] = Name();
        m_outputsNames[2] += "_output_marker_sphere_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[2].c_str());

        m_outputsNames[3] = Name();
        m_outputsNames[3] += "_output_marker_cylinder_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[3].c_str());

        m_outputsNames[4] = Name();
        m_outputsNames[4] += "_output_marker_line_strip_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[4].c_str());

        m_outputsNames[5] = Name();
        m_outputsNames[5] += "_output_marker_line_list_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[5].c_str());

        m_outputsNames[6] = Name();
        m_outputsNames[6] += "_output_marker_cube_list_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[6].c_str());

        m_outputsNames[7] = Name();
        m_outputsNames[7] += "_output_marker_sphere_list_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[7].c_str());

        m_outputsNames[8] = Name();
        m_outputsNames[8] += "_output_marker_points_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[8].c_str());

        m_outputsNames[9] = Name();
        m_outputsNames[9] += "_output_marker_text_view_facing_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[9].c_str());

        m_outputsNames[10] = Name();
        m_outputsNames[10] += "_output_marker_mesh_resource_" + m_inOutName;
        NewOutput("output_real_object", m_outputsNames[10].c_str());

        m_outputsNames[11] = Name();
        m_outputsNames[11] += "_output_marker_triangle_list_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[11].c_str());

        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else//not available
    {

    }
}

bool MAPSros2_topic_marker::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const visualization_msgs::msg::Marker::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_marker::TopicCallbackMarker, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<visualization_msgs::msg::Marker>(m_topicName, 10, fnc);
        Output(m_outputsNames[0].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_ARROW);
        Output(m_outputsNames[1].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_CUBE);
        Output(m_outputsNames[2].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_SPHERE);
        Output(m_outputsNames[3].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_CYLINDER);
        Output(m_outputsNames[4].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_LINE_STRIP);
        Output(m_outputsNames[5].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_LINE_LIST);
        Output(m_outputsNames[6].c_str()).AllocOutputBuffer(1);
        Output(m_outputsNames[7].c_str()).AllocOutputBuffer(1);
        Output(m_outputsNames[8].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_POINTS);
        Output(m_outputsNames[9].c_str()).AllocOutputBuffer(1);
        Output(m_outputsNames[10].c_str()).AllocOutputBuffer(1);
        Output(m_outputsNames[11].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_TRIANGLE_LIST);
    }
    else
    {

    }

    return true;
}

void MAPSros2_topic_marker::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;
}

void MAPSros2_topic_marker::Death()
{
    m_sub.reset();
}

void MAPSros2_topic_marker::TopicCallbackMarker(const visualization_msgs::msg::Marker::SharedPtr marker)
{
    try 
    {
        MAPSTimestamp marker_ts = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            marker_ts = ROSTimeToMAPSTimestamp(marker->header.stamp);
        }

        int concerned_output = m_markers_cache.apply_action(marker);
        auto v = m_markers_cache.get_markers_by_output(concerned_output);

        //for each type run the vector, then StartWriting it in one time
        MarkersWriteByType(v, marker->type, concerned_output, marker_ts);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros2_topic_marker::MarkersWriteByType(std::vector< visualization_msgs::msg::Marker::ConstSharedPtr >& markers, int32_t marker_type, int concerned_output, MAPSTimestamp marker_ts) 
{
    MAPSIOElt *outElt = StartWriting(Output(concerned_output));

    switch (marker_type) 
    {
        case visualization_msgs::msg::Marker::ARROW :
        case visualization_msgs::msg::Marker::CUBE :
        case visualization_msgs::msg::Marker::SPHERE :
        case visualization_msgs::msg::Marker::CYLINDER : {
            // TODO: check buffersize as for POINTS?
            outElt->VectorSize() = MAPSInt32(markers.size());
            int i = 0;
            for (auto &marker : markers) 
            {
                MAPSRealObject &obj = outElt->RealObject(i);
                OutputMarkerOnlyFill(obj, marker);
                i++;
            }
            break;
        }
        case visualization_msgs::msg::Marker::POINTS : 
        {
            int vector_size = 0; // first set output VectorSize
            for (auto& m : markers)
                vector_size += m->points.size();

            if (outElt->BufferSize() < vector_size) 
            {
                StopWriting(outElt);
                MAPSStreamedString ss;
                ss << "Too many points in marker: " << vector_size
                   << " current max:" << outElt->BufferSize();
                ss << " => try increasing it at output allocation.";
                Error(ss);
            }
            outElt->VectorSize() = vector_size;
            int i = 0;
            for (auto& marker : markers) 
            {
                for (auto p : marker->points) 
                {
                    MAPSRealObject &obj = outElt->RealObject(i);
                    OutputMarkerOnlyFill(obj, marker, i);
                    obj.x = p.x;
                    obj.y = p.y;
                    obj.z = p.z;
                    obj.custom.length = marker->scale.y;
                    i++;
                }
            }
            break;
        }
        case visualization_msgs::msg::Marker::LINE_STRIP : 
        {
            int vector_size = 0; // first set output VectorSize
            for (auto& m : markers)
            {
                vector_size += m->points.size() * 3;
            }

            if (outElt->BufferSize() < vector_size) 
            {
                StopWriting(outElt);
                MAPSStreamedString ss;
                ss << "Too many points in marker: " << vector_size
                   << " current max:" << outElt->BufferSize();
                ss << " => try increasing it at output allocation.";
                Error(ss);
            }
            outElt->VectorSize() = vector_size;
            int i = 0;
            for (auto& marker : markers) 
            {
                for (auto p : marker->points) 
                {
                    outElt->Float64(i + 0) = p.x;
                    outElt->Float64(i + 1) = p.y;
                    outElt->Float64(i + 2) = p.z;
                    i += 3;
                }
            }
            break;
        }
        case visualization_msgs::msg::Marker::LINE_LIST : 
        {
            int vector_size = 0; // first set output VectorSize
            for (auto& m : markers)
                vector_size += m->points.size() * 3;

            if (outElt->BufferSize() < vector_size) 
            {
                StopWriting(outElt);
                MAPSStreamedString ss;
                ss << "Too many points in marker: " << vector_size
                   << " current max:" << outElt->BufferSize();
                ss << " => try increasing it at output allocation.";
                Error(ss);
            }
            outElt->VectorSize() = vector_size;
            int i = 0;
            for (auto& marker : markers) 
            {
                for (auto p : marker->points) 
                {
                    outElt->Float64(i + 0) = p.x;
                    outElt->Float64(i + 1) = p.y;
                    outElt->Float64(i + 2) = p.z;
                    i += 3;
                }
            }
            break;
        }
        case visualization_msgs::msg::Marker::CUBE_LIST :
            break;
        case visualization_msgs::msg::Marker::SPHERE_LIST :
            break;
        case visualization_msgs::msg::Marker::TEXT_VIEW_FACING :
            break;
        case visualization_msgs::msg::Marker::MESH_RESOURCE :
            break;
        case visualization_msgs::msg::Marker::TRIANGLE_LIST : {
            int vector_size = 0; // first set output VectorSize
            for (auto &m : markers)
                vector_size += m->points.size() * 3;
            if (outElt->BufferSize() < vector_size) 
            {
                StopWriting(outElt);
                MAPSStreamedString ss;
                ss << "Too many points in marker: " << vector_size
                   << " current max:" << outElt->BufferSize();
                ss << " => try increasing it at output allocation.";
                Error(ss);
            }
            outElt->VectorSize() = vector_size;
            int i = 0;
            for (auto &marker : markers) 
            {
                for (auto p : marker->points) 
                {
                    MAPSRealObject &obj = outElt->RealObject(i);
                    OutputMarkerOnlyFill(obj, marker, i);
                    obj.x = p.x;
                    obj.y = p.y;
                    obj.z = p.z;
                    obj.custom.length = marker->scale.y;
                    i++;
                }
            }
            break;
        }
        default: 
        {
            MAPSStreamedString ss;
            ss << "ROS Marker type unknown: " << marker_type;
            ReportWarning(ss);
            break;
        }
    }
    outElt->Timestamp() = marker_ts;
    StopWriting(outElt);
}

void MAPSros2_topic_marker::OutputMarkerOnlyFill(MAPSRealObject& obj, const visualization_msgs::msg::Marker::ConstSharedPtr& marker, int id)
{
    obj.kind = MAPSRealObject::Custom;
    obj.id = marker->id; // id
    obj.x = marker->pose.position.x;
    obj.y = marker->pose.position.y;
    obj.z = marker->pose.position.z;

    obj.color = MAPS_RGB(MAPSInt64(floor(marker->color.r*255)), MAPSInt64(floor(marker->color.g*255)), MAPSInt64(floor(marker->color.b*255)));
    obj.misc1 = MAPSInt32(0);
    obj.misc2 = MAPSInt32(0);
    obj.misc3 = MAPSInt32(0);

    MAPSCustomRealObjectKind &custom_obj = obj.custom;
    custom_obj.userKind = -1; //MAPSInt32(marker->type);
    custom_obj.width = marker->scale.x;
    custom_obj.height = marker->scale.y;
    custom_obj.length = marker->scale.z;
    custom_obj.phiAngle = marker->pose.orientation.x;
    custom_obj.thetaAngle = marker->pose.orientation.y;
    custom_obj.psiAngle = marker->pose.orientation.z;
}
