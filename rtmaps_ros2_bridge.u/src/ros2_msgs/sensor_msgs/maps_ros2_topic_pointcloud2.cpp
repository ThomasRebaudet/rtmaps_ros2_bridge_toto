#include "ros2_msgs/sensor_msgs/maps_ros2_topic_pointcloud2.h"

void MAPSros2_topic_pointcloud2::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_output_point_cloud_2_info_" + m_inOutName;
        NewOutput("output_int32_array", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_output_point_cloud_2_fields_names_" + m_inOutName;
	    NewOutput("output_text", m_outputsNames[1].c_str());

        m_outputsNames[2] = Name();
        m_outputsNames[2] += "_output_point_cloud_2_fields_info_" + m_inOutName;
	    NewOutput("output_int32_array", m_outputsNames[2].c_str());

        m_outputsNames[3] = Name();
        m_outputsNames[3] += "_output_point_cloud_2_xyz_" + m_inOutName;
	    NewOutput("output_float32_array", m_outputsNames[3].c_str());

        std::string propName  = "max_nb_points_" + m_inOutName;
        NewProperty("max_nb_points", propName.c_str());

        propName = "pointcloud2_nb_additional_fields_" + m_inOutName;
        m_nb_additional_fields = NewProperty("pointcloud2_nb_additional_fields", propName.c_str()).IntegerValue();
        m_additional_fields.resize(m_nb_additional_fields);

        int current_offset = 0;
        for (int i=0; i<m_nb_additional_fields; i++)
        {
            MAPSStreamedString prefix;
            prefix << "field_" << i+4;
            MAPSStreamedString field_name_propname;
            MAPSStreamedString field_type_propname;
            field_name_propname << prefix << "_name";
            field_type_propname << prefix << "_datatype";
            NewProperty("pointcloud2_field_name",field_name_propname);
            NewProperty("pointcloud2_field_data_type",field_type_propname);
            MAPSString field_name = GetStringProperty(field_name_propname);
            int field_type = static_cast<int>(GetIntegerProperty(field_type_propname));
            m_additional_fields[i].name = static_cast<const char*>(field_name);
            m_additional_fields[i].count = 1;
            m_additional_fields[i].offset = current_offset;
            if (field_name.Len() > 0)
            {
                MAPSStreamedString output_name;
                output_name << prefix << "_" << field_name;
                switch(field_type)
                {
                    case 0: //INT8
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::INT8;
                        current_offset += sizeof(int8_t);
                        NewOutput("output_int32_array", output_name);
                    break;
                    case 1: //UINT8
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::UINT8;
                        current_offset += sizeof(uint8_t);
                        NewOutput("output_int32_array", output_name);
                        break;
                    case 2: //INT16
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::INT16;
                        current_offset += sizeof(int16_t);
                        NewOutput("output_int32_array", output_name);
                        break;
                    case 3: //UINT16
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::UINT16;
                        current_offset += sizeof(uint16_t);
                        NewOutput("output_int32_array", output_name);
                        break;
                    case 4: //INT32
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::INT32;
                        current_offset += sizeof(int32_t);
                        NewOutput("output_int32_array", output_name);
                        break;
                    case 5: //UINT32
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::UINT32;
                        current_offset += sizeof(uint32_t);
                        NewOutput("output_int64_array", output_name);
                        break;
                    case 6: //FLOAT32
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
                        current_offset += sizeof(MAPSFloat32);
                        NewOutput("output_float32_array", output_name);
                        break;
                    case 7: //FLOAT64
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::FLOAT64;
                        current_offset += sizeof(MAPSFloat64);
                        NewOutput("output_float64_array", output_name);
                    break;
                }
            }
        }

        propName = "transfer_ROS_timestamps_" + m_inOutName;
        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps", propName.c_str()).BoolValue();
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_" + m_inOutName;
        NewInput("input_pointcloud_xyz", m_inputName.c_str());

        std::string s = "pointcloud2_width_" + m_inOutName;
        NewProperty("pointcloud2_width", s.c_str());
        s = "pointcloud2_height_" + m_inOutName;
        NewProperty("pointcloud2_height", s.c_str());
        s = "pointcloud2_is_dense_" + m_inOutName;
        NewProperty("pointcloud2_is_dense", s.c_str());
        s = "pointcloud2_datatype_" + m_inOutName;
        NewProperty("pointcloud2_datatype", s.c_str());

        s = "pointcloud2_nb_additional_fields_" + m_inOutName;
        m_nb_additional_fields = NewProperty("pointcloud2_nb_additional_fields", s.c_str()).IntegerValue();
        m_additional_fields.resize(m_nb_additional_fields);
        int current_offset = 0;
        for (int i=0; i<m_nb_additional_fields; i++)
        {
            MAPSStreamedString prefix;
            prefix << "field_" << i+4;
            MAPSStreamedString field_name_propname;
            MAPSStreamedString field_type_propname;
            field_name_propname << prefix << "_name";
            field_type_propname << prefix << "_datatype";
            NewProperty("pointcloud2_field_name",field_name_propname);
            NewProperty("pointcloud2_field_data_type",field_type_propname);
            MAPSString field_name = GetStringProperty(field_name_propname);
            int field_type = static_cast<int>(GetIntegerProperty(field_type_propname));
            m_additional_fields[i].name = static_cast<const char*>(field_name);
            m_additional_fields[i].count = 1;
            m_additional_fields[i].offset = current_offset;
            if (field_name.Len() > 0)
            {
                MAPSStreamedString input_name;
                input_name << prefix << "_" << field_name;
                switch(field_type)
                { //"INT8|UINT8|INT16|UINT16|INT32|UINT32|FLOAT32|FLOAT64"
                    case 0: //INT8
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::INT8;
                        current_offset += sizeof(int8_t);
                        NewInput("input_int32", input_name);
                        break;
                    case 1: //UINT8
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::UINT8;
                        current_offset += sizeof(uint8_t);
                        NewInput("input_int32", input_name);
                        break;
                    case 2: //INT16
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::INT16;
                        current_offset += sizeof(int16_t);
                        NewInput("input_int32", input_name);
                        break;
                    case 3: //UINT16
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::UINT16;
                        current_offset += sizeof(uint16_t);
                        NewInput("input_int32", input_name);
                        break;
                    case 4: //INT32
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::INT32;
                        current_offset += sizeof(int32_t);
                        NewInput("input_int32", input_name);
                        break;
                    case 5: //UINT32
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::UINT32;
                        current_offset += sizeof(uint32_t);
                        NewInput("input_int64", input_name);
                        break;
                    case 6: //FLOAT32
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
                        current_offset += sizeof(MAPSFloat32);
                        NewInput("input_float32", input_name);
                        break;
                    case 7:
                        m_additional_fields[i].datatype = sensor_msgs::msg::PointField::FLOAT64;
                        current_offset += sizeof(MAPSFloat64);
                        NewInput("input_float64", input_name);
                        break;
                }
            }
        }

        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }
}

bool MAPSros2_topic_pointcloud2::OutputsSanityCheck()
{
    for (int i=0; i<m_nb_additional_fields; i++)
    {
        if (m_additional_fields[i].name.length() == 0)
        {
            MAPSStreamedString ss;
            ss << "Field " << i+4 << " name is empty. Please provide a field name.";
            Error(ss);
            return false;
        }
    }
    return true;
}

bool MAPSros2_topic_pointcloud2::InputsSanityCheck()
{
    for (int i=0; i<m_nb_additional_fields; i++)
    {
        if (m_additional_fields[i].name.length() == 0)
        {
            MAPSStreamedString ss;
            ss << "Field " << i+4 << " name is empty. Please provide a field name.";
            Error(ss);
            return false;
        }
    }
    return true;
}

bool MAPSros2_topic_pointcloud2::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        if (!OutputsSanityCheck())
        {
            return false;
        }

        std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_pointcloud2::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::PointCloud2>(m_topicName, 10, fnc);
    }
    else
    {
        if (!InputsSanityCheck())
        {
            return false;
        }

        m_nb_inputs = 1 + m_nb_additional_fields;
        if (m_nb_inputs > MAX_NB_FIELDS)
        {
            MAPSStreamedString ss;
            ss << "Too many additional fields.";
            Error(ss);
            return false;
        }
        for (int i=0; i<m_nb_inputs; i++)
        {
            m_inputs[i] = &Input(i);
        }

        m_pub = m_node->create_publisher<sensor_msgs::msg::PointCloud2>(m_topicName, 100);
    }

    m_firstTime = true;

    return true;
}

void MAPSros2_topic_pointcloud2::AllocOutputBuffers(int32_t nb_fields, int32_t nb_additional_fields, int32_t max_nb_points)
{
    Output(m_outputsNames[0].c_str()).AllocOutputBuffer(6);
    Output(m_outputsNames[1].c_str()).AllocOutputBuffer(nb_fields*256); //255 chars max per field name.
    Output(m_outputsNames[2].c_str()).AllocOutputBuffer(nb_fields*3); //(offset, datatype, count)
    Output(m_outputsNames[3].c_str()).AllocOutputBuffer(max_nb_points * 3); //data for x, y, z fields

    for (int i = 0; i < nb_additional_fields; i++)
    {
        Output(4+i).AllocOutputBuffer(max_nb_points); //data for additional fields (intensity, ring, reflectivy, r, g, b, etc.)
    }

}

template <typename T> void MAPSros2_topic_pointcloud2::OutputXYZ(MAPSFloat32* dst, const uint8_t* src, int nb_points, int point_step)
{
    const T* typed_src = reinterpret_cast<const T*> (src);
    for (int i=0; i<nb_points; i++)
    {
        *(dst++) = *(typed_src++);
        *(dst++) = *(typed_src++);
        *(dst++) = *typed_src;
        src += point_step;
        typed_src = reinterpret_cast<const T*> (src);
    }
}

template <typename T, typename U> void MAPSros2_topic_pointcloud2::OutputField(T* dst, const uint8_t* src, int nb_points, int offset, int point_step)
{
    src += offset;
    const U* typed_src = reinterpret_cast<const U*> (src);
    for (int i=0; i<nb_points; i++)
    {
        *(dst++) = *typed_src;
        src += point_step;
        typed_src = reinterpret_cast<const U*> (src);
    }
}

template <typename T_DEST, typename T_SRC> void MAPSros2_topic_pointcloud2::CopyXYZ(uint8_t* dst, void* src, int nb_points, int point_step)
{
    T_DEST* out_point = reinterpret_cast<T_DEST*> (dst);
    T_SRC* in_point = reinterpret_cast<T_SRC*> (src);
    for (int i=0; i<nb_points; i++)
    {
        *(out_point++) = static_cast<T_DEST> (*(in_point++)); //X
        *(out_point++) = static_cast<T_DEST> (*(in_point++)); //Y
        *(out_point++) = static_cast<T_DEST> (*(in_point++)); //Z
        dst += point_step;
        out_point = reinterpret_cast<T_DEST*>(dst);
    }
}

template <typename T_DEST, typename T_SRC> void MAPSros2_topic_pointcloud2::CopyAdditionalField(uint8_t* dst, void* src, int nb_points, int offset, int point_step)
{
    dst += offset;
    T_DEST* out_field_data = reinterpret_cast<T_DEST*>(dst);
    T_SRC* in_field_data = reinterpret_cast<T_SRC*>(src);
    for (int i=0; i<nb_points; i++)
    {
        *(out_field_data) = *(in_field_data++);
        dst += point_step;
        out_field_data = reinterpret_cast<T_DEST*>(dst);
    }
}

bool MAPSros2_topic_pointcloud2::BuildPointCloudModel(MAPSTypeInfo& input_type_xyz)
{
    m_pointcloud_model.fields.resize(3 + m_nb_additional_fields);
    m_pointcloud_model.fields[0].name = "x";
    m_pointcloud_model.fields[1].name = "y";
    m_pointcloud_model.fields[2].name = "z";

    m_pointcloud_model.fields[0].count = 1;
    m_pointcloud_model.fields[1].count = 1;
    m_pointcloud_model.fields[2].count = 1;

    std::string propName = "pointcloud2_datatype_" + m_inOutName;
    m_ros_pointcloud2_output_type = (int)GetIntegerProperty(propName.c_str());
    propName = "pointcloud2_is_dense_" + m_inOutName;
    m_ros_pointcloud2_is_dense = GetBoolProperty(propName.c_str());
    m_ros_pointcloud2_is_bigendian = false;

    if (MAPS::TypeFilter(input_type_xyz,MAPS::FilterInteger64))
    {
        Error("64 bit integer point clouds are not supported in ROS.");
        return false;
    }
    propName = "pointcloud2_width_" + m_inOutName;
    m_width = static_cast<int>(GetIntegerProperty(propName.c_str()));
    propName = "pointcloud2_height_" + m_inOutName;
    m_height = static_cast<int>(GetIntegerProperty(propName.c_str()));

    int current_offset = 0;
    if(m_ros_pointcloud2_output_type == 1 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(input_type_xyz,MAPS::FilterInteger32))) 
    {
        m_pointcloud_model.fields[0].offset = 0;
        m_pointcloud_model.fields[1].offset = 4;
        m_pointcloud_model.fields[2].offset = 8;
        current_offset = 12;
        m_pointcloud_model.point_step = 4 * 3;
        m_pointcloud_model.fields[0].datatype = sensor_msgs::msg::PointField::INT32;
        m_pointcloud_model.fields[1].datatype = sensor_msgs::msg::PointField::INT32;
        m_pointcloud_model.fields[2].datatype = sensor_msgs::msg::PointField::INT32;
    } 
    else if (m_ros_pointcloud2_output_type == 2 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(input_type_xyz,MAPS::FilterFloat32))) 
    {
        m_pointcloud_model.point_step = 4 * 3;
        m_pointcloud_model.fields[0].offset = 0;
        m_pointcloud_model.fields[1].offset = 4;
        m_pointcloud_model.fields[2].offset = 8;
        current_offset = 12;
        m_pointcloud_model.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_pointcloud_model.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_pointcloud_model.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    } 
    else if (m_ros_pointcloud2_output_type == 3 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(input_type_xyz,MAPS::FilterFloat64))) 
    {
        m_pointcloud_model.point_step = 8 * 3;
        m_pointcloud_model.fields[0].offset = 0;
        m_pointcloud_model.fields[1].offset = 8;
        m_pointcloud_model.fields[2].offset = 16;
        current_offset = 24;
        m_pointcloud_model.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
        m_pointcloud_model.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT64;
        m_pointcloud_model.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT64;
    }        

    //Add the additional fields (other than x, y, z) to the description.
    for (int i=0; i< m_nb_additional_fields; i++)
    {
        m_pointcloud_model.fields[3+i].count = 1;
        m_pointcloud_model.fields[3+i].name = m_additional_fields[i].name;
        m_pointcloud_model.fields[3+i].datatype = m_additional_fields[i].datatype;
        m_pointcloud_model.fields[3+i].offset = current_offset + m_additional_fields[i].offset;
        switch (m_additional_fields[i].datatype)
        {
            case sensor_msgs::msg::PointField::INT8:
            case sensor_msgs::msg::PointField::UINT8:
                m_pointcloud_model.point_step += sizeof(int8_t);
                break;
            case sensor_msgs::msg::PointField::INT16:
            case sensor_msgs::msg::PointField::UINT16:
                m_pointcloud_model.point_step += sizeof(int16_t);
                break;
            case sensor_msgs::msg::PointField::UINT32:
            case sensor_msgs::msg::PointField::INT32:
                m_pointcloud_model.point_step += sizeof(int32_t);
            break;
            case sensor_msgs::msg::PointField::FLOAT32:
                m_pointcloud_model.point_step += sizeof(MAPSFloat32);
                break;
            case sensor_msgs::msg::PointField::FLOAT64:
                m_pointcloud_model.point_step += sizeof(MAPSFloat64);
                break;
            default:
                Error("Unsupported data type for additional field.");
                return false;
        }
    }

    m_pointcloud_model.row_step = m_pointcloud_model.point_step * m_pointcloud_model.width;

    return true;
}

void MAPSros2_topic_pointcloud2::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSTimestamp t = SynchroStartReading(m_nb_inputs,m_inputs,m_ioelts);
	if (t < 0)
		return;

    if (m_firstTime) 
    {
        m_firstTime = false;
        BuildPointCloudModel(m_ioelts[0]->Type());
    }

    sensor_msgs::msg::PointCloud2 ros_pointcloud2(m_pointcloud_model);

    if (m_ioelts[0]->VectorSize() % 3 != 0) 
    {
        Error("The input vector size is not a multiple of 3 as expected for an XYZ point cloud.");
        return;
    }
    int ros_pointcloud2_nb_points_in = m_ioelts[0]->VectorSize() / 3;
    for (int i=1; i<m_nb_inputs; i++)
    {
        if (m_ioelts[i]->VectorSize() != ros_pointcloud2_nb_points_in)
        {
            MAPSStreamedString ss;
            ss << "The vector size received for field " << i+4 << " (" << m_additional_fields[i].name.c_str() << ") does not match the number of x, y, z points received on first input. Discarding...";
            ReportError(ss);
            return;
        }
    }

    if (m_width > 0 && m_height > 0) 
    {
        if (ros_pointcloud2_nb_points_in != m_width * m_height) 
        {
            MAPSStreamedString ss;
            ss << "The input number of points (" << ros_pointcloud2_nb_points_in << ") does not match the dimensions of the publised PointCloud2 (" << m_width << "x" << m_height << ")";
            Error(ss);
            return;
        }
        ros_pointcloud2.width = m_width;
        ros_pointcloud2.height = m_height;
    } 
    else 
    {
        if (m_width <= 0 && m_height <= 0) 
        {
            ros_pointcloud2.width = ros_pointcloud2_nb_points_in;
            ros_pointcloud2.height = 1;
        } else if (m_width <= 0) {
            ros_pointcloud2.height = m_height;
            ros_pointcloud2.width = ros_pointcloud2_nb_points_in/ros_pointcloud2.height;
        } else if (m_height <= 0) {
            ros_pointcloud2.width = m_width;
            ros_pointcloud2.height = ros_pointcloud2_nb_points_in/ros_pointcloud2.width;
        }
    }
    ros_pointcloud2.row_step = ros_pointcloud2.point_step * ros_pointcloud2.width;
    unsigned int data_size = ros_pointcloud2.row_step * ros_pointcloud2.height;
    if (ros_pointcloud2.data.size() != data_size) 
    {
        ros_pointcloud2.data.resize(ros_pointcloud2.row_step * ros_pointcloud2.height);
    }

    if (m_ros_pointcloud2_output_type == 1 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(m_ioelts[0]->Type(),MAPS::FilterInteger32)) ) 
    {
        //Output an int32 point cloud.
        if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterInteger32))
        {
            CopyXYZ<MAPSInt32, MAPSInt32>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
        else if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterFloat32))
        {
            CopyXYZ<MAPSInt32, MAPSFloat32>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
        else if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterFloat64))
        {
            CopyXYZ<MAPSInt32, MAPSFloat64>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
    }
    else if (m_ros_pointcloud2_output_type == 2 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(m_ioelts[0]->Type(),MAPS::FilterFloat32)) )
    {
        //Output a float32 point cloud.
        if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterInteger32))
        {
            CopyXYZ<MAPSFloat32, MAPSInt32>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
        else if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterFloat32))
        {
            CopyXYZ<MAPSFloat32, MAPSFloat32>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
        else if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterFloat64))
        {
            CopyXYZ<MAPSFloat32, MAPSFloat64>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
    }
    else if (m_ros_pointcloud2_output_type == 3 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(m_ioelts[0]->Type(),MAPS::FilterFloat64)) )
    {
        //Output a float32 point cloud.
        if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterInteger32))
        {
            CopyXYZ<MAPSFloat64, MAPSInt32>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
        else if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterFloat32))
        {
            CopyXYZ<MAPSFloat64, MAPSFloat32>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
        else if (MAPS::TypeFilter(m_ioelts[0]->Type(), MAPS::FilterFloat64))
        {
            CopyXYZ<MAPSFloat64, MAPSFloat64>(ros_pointcloud2.data.data(), m_ioelts[0]->Data(), ros_pointcloud2_nb_points_in, ros_pointcloud2.point_step);
        } 
    }
    else 
    {
        Error("The requested data type conversion is not supported yet.");
        return;
    }

    //Fill in the additional fields
    for (int i=0; i<m_nb_additional_fields; i++)
    {
        void* src = m_ioelts[i+1]->Data();
        switch(m_additional_fields[i].datatype)
        {
            case sensor_msgs::msg::PointField::INT8:
            {
                CopyAdditionalField<int8_t, MAPSInt32>(ros_pointcloud2.data.data(), src, ros_pointcloud2_nb_points_in, ros_pointcloud2.fields[i+3].offset, ros_pointcloud2.point_step);
            }
            break;
            case sensor_msgs::msg::PointField::UINT8:
            {
                CopyAdditionalField<uint8_t, MAPSInt32>(ros_pointcloud2.data.data(), src, ros_pointcloud2_nb_points_in, ros_pointcloud2.fields[i+3].offset, ros_pointcloud2.point_step);
            }
            break;
            case sensor_msgs::msg::PointField::INT16:
            {
                CopyAdditionalField<int16_t, MAPSInt32>(ros_pointcloud2.data.data(), src, ros_pointcloud2_nb_points_in, ros_pointcloud2.fields[i+3].offset, ros_pointcloud2.point_step);
            }
            break;
            case sensor_msgs::msg::PointField::UINT16:
            {
                CopyAdditionalField<uint16_t, MAPSInt32>(ros_pointcloud2.data.data(), src, ros_pointcloud2_nb_points_in, ros_pointcloud2.fields[i+3].offset, ros_pointcloud2.point_step);
            }
            break;
            case sensor_msgs::msg::PointField::INT32:
            {
                CopyAdditionalField<int32_t, MAPSInt32>(ros_pointcloud2.data.data(), src, ros_pointcloud2_nb_points_in, ros_pointcloud2.fields[i+3].offset, ros_pointcloud2.point_step);
            }
            break;
            case sensor_msgs::msg::PointField::UINT32:
            {
                CopyAdditionalField<uint32_t, MAPSInt64>(ros_pointcloud2.data.data(), src, ros_pointcloud2_nb_points_in, ros_pointcloud2.fields[i+3].offset, ros_pointcloud2.point_step);
            }
            break;
            case sensor_msgs::msg::PointField::FLOAT32:
            {
                CopyAdditionalField<float, MAPSFloat32>(ros_pointcloud2.data.data(), src, ros_pointcloud2_nb_points_in, ros_pointcloud2.fields[i+3].offset, ros_pointcloud2.point_step);
            }
            break;
            case sensor_msgs::msg::PointField::FLOAT64:
            {
                CopyAdditionalField<double, MAPSFloat64>(ros_pointcloud2.data.data(), src, ros_pointcloud2_nb_points_in, ros_pointcloud2.fields[i+3].offset, ros_pointcloud2.point_step);
            }
            break;
            default:
            {
                MAPSStreamedString ss;
                ss << "Unsuppored data type for field " << m_additional_fields[i].name.c_str();
                Error(ss);
                return;
            }
        }
    }

    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    ros_pointcloud2.header = m_header;
    m_pub->publish(ros_pointcloud2);
}

void MAPSros2_topic_pointcloud2::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_pointcloud2::TopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ros_point_cloud2)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(ros_point_cloud2->header.stamp);
        }

        if (m_firstTime) 
        {
            m_nb_fields = ros_point_cloud2->fields.size();
            //Check we get at least the x, y, z fields.
            if (m_nb_fields < 3)
            {
                ReportError("Number of fields seems to be lower than 3. Expecting at least the x, y, and z fields.");
                return;
            }
            m_firstTime = false;

            m_point_step = ros_point_cloud2->point_step;
            m_point_datatype = ros_point_cloud2->fields[0].datatype; //Let's assume at least X, Y and Z fields have the same datatype...
            m_nb_points = ros_point_cloud2->data.size()/ros_point_cloud2->point_step;

            std::string propName = "max_nb_points_" + m_inOutName;
            int max_nb_points = (int)GetIntegerProperty(propName.c_str());
            
            if (ros_point_cloud2->fields[0].name != "x" ||
                ros_point_cloud2->fields[1].name != "y" ||
                ros_point_cloud2->fields[2].name != "z")
            {
                MAPSStreamedString ss;
                ss <<"Couldn't find the x, y and z fields as first 3 fields in received PointCloud2 message. Unsupported fields layout." << MAPSSManip::endl;
                ss <<"Found 3 first fields: " << ros_point_cloud2->fields[0].name.c_str() << " - ";
                ss << ros_point_cloud2->fields[1].name.c_str() << " - ";
                ss << ros_point_cloud2->fields[2].name.c_str(); 
                Error(ss);
            }
            //Check that the additional fields defined in the component properties match the received fields in the PointCloud2 message.
            if (m_nb_additional_fields > (m_nb_fields - 3))
            {
                MAPSStreamedString ss;
                ss << "The number of additional fields in the received point cloud - after the x, y and z fields - (" << m_nb_fields - 3;
                ss << ") is lower than the expected number of additional fields declared in the component properties (" << m_nb_additional_fields << ").";
                Error(ss);
                return;
            }

            bool field_defs_ok = true;
            for (int i = 0; i < m_nb_additional_fields; i++)
            {
                if (ros_point_cloud2->fields[i+3].name != m_additional_fields[i].name)
                {
                    MAPSStreamedString ss;
                    ss << "Field " << i+3 << ": expecting field name " << m_additional_fields[i].name.c_str();
                    ss << " but received " << ros_point_cloud2->fields[i+3].name.c_str() <<". Aborting.";
                    ReportError(ss);
                    field_defs_ok = false;
                }
                if (ros_point_cloud2->fields[i+3].datatype != m_additional_fields[i].datatype)
                {
                    MAPSStreamedString ss;
                    ss << "Field " << i+3 << ": expecting field datatype " << m_additional_fields[i].datatype;
                    ss << " but received " << ros_point_cloud2->fields[i+3].datatype << ". Aborting.";
                    ReportError(ss);
                    field_defs_ok = false;
                }
            }

            if (false == field_defs_ok)
            {
                Error("Mismatch detected between received and defined field definitions.");
                return;
            }

            if (max_nb_points > 0) 
            {
                if (m_nb_points > max_nb_points) 
                {
                    MAPSStreamedString ss;
                    ss << "Number of points if first packet (" << m_nb_points << ") is higher than max_nb_points property (" << max_nb_points << "). Using number of points in first packet received.";
                    ReportWarning(ss);
                    max_nb_points = m_nb_points;
                }
                m_nb_points = max_nb_points;
            }

            AllocOutputBuffers(m_nb_fields, m_nb_additional_fields, m_nb_points);
        }

        MAPSIOElt* out_info = NULL, *out_field_names = NULL, *out_fields_info = NULL, *out_data = NULL;
        out_info = StartWriting(Output(0));
        out_field_names = StartWriting(Output(1));
        out_fields_info = StartWriting(Output(2));
        out_data = StartWriting(Output(3));

        out_info->Integer32(0) = ros_point_cloud2->width;
        out_info->Integer32(1) = ros_point_cloud2->height;
        out_info->Integer32(2) = ros_point_cloud2->is_bigendian ? 1 : 0;
        out_info->Integer32(3) = ros_point_cloud2->point_step;
        out_info->Integer32(4) = ros_point_cloud2->row_step;
        out_info->Integer32(5) = ros_point_cloud2->is_dense ? 1 : 0;

        int nb_fields = ros_point_cloud2->fields.size();
        if (nb_fields > m_nb_fields) 
        {
            MAPSStreamedString ss;
            ss << "Number of fields has increased. Problem. Truncating.";
            ReportWarning(ss);
            nb_fields = m_nb_fields;
        }

        MAPSString field_names;
        for (int i=0; i<nb_fields; i++) 
        {
            field_names += (const char*)ros_point_cloud2->fields[i].name.c_str();
            field_names += "|";
        }
        if (field_names.Len() >= 255*nb_fields) 
        {
            MAPSStreamedString ss;
            ss << "Field names too long. Problem. Truncating.";
            ReportWarning(ss);
            field_names = field_names.Left(255*nb_fields);
        }
        MAPS::Strcpy(out_field_names->TextAscii(),(const char*)field_names);
        out_field_names->VectorSize() = field_names.Len();

        for (int i=0; i<nb_fields; i++) 
        {
            out_fields_info->Integer32(3*i) = ros_point_cloud2->fields[i].offset;
            out_fields_info->Integer32(3*i+1) = ros_point_cloud2->fields[i].datatype;
            out_fields_info->Integer32(3*i+2) = ros_point_cloud2->fields[i].count;
        }
        out_fields_info->VectorSize() = nb_fields*3;

        int nb_points = ros_point_cloud2->data.size() / ros_point_cloud2->point_step;
        if (nb_points > m_nb_points) 
        {
            MAPSStreamedString ss;
            ss << "Amount of points (" << nb_points << ") exceeds maximum number of points (" << m_nb_points << "). Truncating. If property max_nb_points is set to -1, it means that number of points has increased since first sample. Consider increasing max_nb_points.";
            ReportWarning(ss);
            nb_points = m_nb_points;
        }
        MAPSFloat32* data_out = &out_data->Float32();
        if (m_point_datatype == sensor_msgs::msg::PointField::INT32) //Int32
        {
            OutputXYZ<int32_t>(data_out, ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->point_step);
        } 
        else if (m_point_datatype == sensor_msgs::msg::PointField::FLOAT32) 
        { //Float32
            OutputXYZ<MAPSFloat32>(data_out, ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->point_step);
        } 
        else if (m_point_datatype == 8) //Float64
        {
            OutputXYZ<MAPSFloat64>(data_out, ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->point_step);
        }
        else
        {
            MAPSStreamedString ss;
            ss << "Input point cloud 2 has data type " << m_point_datatype << " which is not suppored yet.";
            Error(ss);
            return;
        }
        out_data->VectorSize() = nb_points * 3;


        out_info->Timestamp() = t;
        out_field_names->Timestamp() = t;
        out_fields_info->Timestamp() = t;
        out_data->Timestamp() = t;

        StopWriting(out_data);
        StopWriting(out_fields_info,nb_fields==0);
        StopWriting(out_field_names,nb_fields==0);
        StopWriting(out_info);

        //Output additional fields.
        for (int i=0; i< m_nb_additional_fields; i++)
        {
            MAPSIOElt* ioeltout = StartWriting(Output(4+i));
            switch(ros_point_cloud2->fields[3+i].datatype)
            {
                case sensor_msgs::msg::PointField::INT8:
                {
                    OutputField<int32_t, int8_t>(&ioeltout->Integer32(), ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->fields[3+i].offset, ros_point_cloud2->point_step);
                }
                break;
                case sensor_msgs::msg::PointField::UINT8:
                {
                    OutputField<int32_t, uint8_t>(&ioeltout->Integer32(), ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->fields[3+i].offset, ros_point_cloud2->point_step);
                }
                break;
                case sensor_msgs::msg::PointField::INT16:
                {
                    OutputField<int32_t, int16_t>(&ioeltout->Integer32(), ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->fields[3+i].offset, ros_point_cloud2->point_step);
                }
                break;
                case sensor_msgs::msg::PointField::UINT16:
                {
                    OutputField<int32_t, uint16_t>(&ioeltout->Integer32(), ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->fields[3+i].offset, ros_point_cloud2->point_step);
                }
                break;
                case sensor_msgs::msg::PointField::INT32:
                {
                    OutputField<int32_t, int32_t>(&ioeltout->Integer32(), ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->fields[3+i].offset, ros_point_cloud2->point_step);
                }
                break;
                case sensor_msgs::msg::PointField::UINT32:
                {
                    OutputField<int64_t, uint32_t>(&ioeltout->Integer64(), ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->fields[3+i].offset, ros_point_cloud2->point_step);
                }
                break;
                case sensor_msgs::msg::PointField::FLOAT32:
                {
                    OutputField<MAPSFloat32,MAPSFloat32>(&ioeltout->Float32(), ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->fields[3+i].offset, ros_point_cloud2->point_step);
                }
                break;
                case sensor_msgs::msg::PointField::FLOAT64:
                {
                    OutputField<MAPSFloat64,MAPSFloat64>(&ioeltout->Float64(), ros_point_cloud2->data.data(), nb_points, ros_point_cloud2->fields[3+i].offset, ros_point_cloud2->point_step);
                }
                break;
                default:
                {
                    MAPSStreamedString ss;
                    ss << "Input point cloud 2 has field " << ros_point_cloud2->fields[3+i].name.c_str() << " which data type " << ros_point_cloud2->fields[3+i].datatype << " is not supported yet. Discarding...";
                    Error(ss);
                    StopWriting(ioeltout,true);
                    return;
                }
            }
            ioeltout->Timestamp() = t;
            ioeltout->VectorSize() = nb_points;
            StopWriting(ioeltout);
        }
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}