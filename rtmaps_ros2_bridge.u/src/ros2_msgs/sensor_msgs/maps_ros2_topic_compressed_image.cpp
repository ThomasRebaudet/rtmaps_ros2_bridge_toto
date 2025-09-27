#include "ros2_msgs/sensor_msgs/maps_ros2_topic_compressed_image.h"

void MAPSros2_topic_compressed_image::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_" + m_inOutName;
        NewOutput("output_compressed_image", m_outputName.c_str());
        std::string s = "compressed_image_width_" + m_inOutName;
        m_width = NewProperty("compressed_image_width", s.c_str()).IntegerValue();
        s = "compressed_image_height_" + m_inOutName;
        m_height = NewProperty("compressed_image_height", s.c_str()).IntegerValue();
        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_" + m_inOutName;
        NewInput("input_mapsimage", m_inputName.c_str());
        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }
}

bool MAPSros2_topic_compressed_image::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_compressed_image::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::CompressedImage>(m_topicName, 10, fnc);
    }
    else
    {
        m_pub = m_node->create_publisher<sensor_msgs::msg::CompressedImage>(m_topicName, 100);
    }

    m_firstTime = true;

    return true;
}

void MAPSros2_topic_compressed_image::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    MAPSImage& image_in = ioeltin->MAPSImage();
    if (m_firstTime) 
    {
        m_firstTime = false;

        if(*(MAPSUInt32*)image_in.imageCoding != MAPS_IMAGECODING_JPEG &&
            *(MAPSUInt32*)image_in.imageCoding != MAPS_IMAGECODING_PNG) 
        {
            Error("Unsupported format for compressed image. ROS supports only jpeg and png images.");
        }

        switch(*(MAPSUInt32*)image_in.imageCoding) 
        {
            case MAPS_IMAGECODING_JPEG:
                m_message.format = "jpeg";
                break;
            case MAPS_IMAGECODING_PNG:
                m_message.format = "png";
                break;
            default:
                Error("Unsupported image format.");
        }
    }

    MAPSTimestamp t = ioeltin->Timestamp();
    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    m_message.header = m_header;
    m_message.data.resize(image_in.imageSize);
    MAPS::Memcpy((char*)&m_message.data[0],image_in.imageData,image_in.imageSize);
    m_pub->publish(m_message);
}

void MAPSros2_topic_compressed_image::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_compressed_image::TopicCallback(const sensor_msgs::msg::CompressedImage::SharedPtr ros_image)
{
    try 
    {
        int32_t size = MIN((int32_t)ros_image->data.size(), m_width * m_height * 4);

        if (m_firstTime) 
        {
            m_firstTime = false;
            MAPSUInt32 encoding = MAPS_IMAGECODING_UNKNOWN;
            if (ros_image->format.find("jpeg")  != std::string::npos)
            {
                encoding = MAPS_IMAGECODING_JPEG;
            }
            else if (ros_image->format.find("png") != std::string::npos)
            {
                encoding = MAPS_IMAGECODING_PNG;
            }
            else 
            {
                if (m_sub)
                {
                    m_sub.reset(); // stop topic subscription
                }
                MAPSStreamedString ss;
                ss << "Image format not supported: " << (const char *) ros_image->format.c_str();
                Error(ss);
                return;
            }

            MAPSStreamedString ss;
            ss << "Max output image size : " << size << ": " << m_width << "x" << m_height;
            ss << ", image encoding: " << ros_image->format.c_str();
            ReportInfo(ss);

            Output(m_outputName.c_str()).AllocOutputBufferMAPSImage(size, m_width, m_height, encoding);
        }

        MAPSIOElt* ioeltout = StartWriting(Output(m_outputName.c_str()));

        if(m_transferRosTimestamp)
        {
            ioeltout->Timestamp() = ROSTimeToMAPSTimestamp(ros_image->header.stamp);
        }
        else
        {
            ioeltout->Timestamp() = MAPS::CurrentTime();
        }

        MAPSImage &image_out = ioeltout->MAPSImage();
        MAPS::Memcpy(image_out.imageData, (char *) &ros_image->data[0], size);
        image_out.imageSize = (int32_t)ros_image->data.size();
        image_out.bufferSize = (int32_t)ros_image->data.size();
        ioeltout->VectorSize() = 1;
        StopWriting(ioeltout);

    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}