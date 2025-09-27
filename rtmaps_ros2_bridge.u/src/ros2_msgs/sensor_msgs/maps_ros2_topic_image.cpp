#include "ros2_msgs/sensor_msgs/maps_ros2_topic_image.h"

void MAPSros2_topic_image::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_" + m_inOutName;
        NewOutput("output_image", m_outputName.c_str());
        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_" + m_inOutName;
        NewInput("input_image", m_inputName.c_str());
        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }

}

bool MAPSros2_topic_image::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const sensor_msgs::msg::Image::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_image::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::Image>(m_topicName, 10, fnc);
    }
    else
    {
        m_pub = m_node->create_publisher<sensor_msgs::msg::Image>(m_topicName, 100);
    }

    m_firstTime = true;

    return true;
}

void MAPSros2_topic_image::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    IplImage& image_in = ioeltin->IplImage();
    if (m_firstTime) 
    {
        m_firstTime = false;
        m_message.height = image_in.height;
        m_message.width = image_in.width;
        m_message.is_bigendian = false;
        switch(*(MAPSUInt32*)image_in.channelSeq) 
        {
            case MAPS_CHANNELSEQ_BGR:
                m_message.encoding = "bgr8";
                break;
            case MAPS_CHANNELSEQ_RGB:
                m_message.encoding = "rgb8";
                break;
            case MAPS_CHANNELSEQ_GRAY:
                m_message.encoding = "mono8";
                break;
            case MAPS_CHANNELSEQ_RGBA:
                m_message.encoding = "rgba8";
                break;
            case MAPS_CHANNELSEQ_BGRA:
                m_message.encoding = "bgra8";
                break;
            default:
                Error("Unsupported image format");
        }
        m_message.step = image_in.widthStep;
        m_message.data.resize(image_in.imageSize);
    }

    MAPSTimestamp t = ioeltin->Timestamp();
    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    m_message.header = m_header;

    MAPS::Memcpy((char*)&m_message.data[0], image_in.imageData, image_in.imageSize);
    m_pub->publish(m_message);
}

void MAPSros2_topic_image::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_image::TopicCallback(const sensor_msgs::msg::Image::SharedPtr ros_image)
{
    try 
    {
        if (m_firstTime) 
        {
            m_firstTime = false;
            MAPSUInt32 chanseq = MAPS_CHANNELSEQ_RGBA, depth = 8;
            if (ros_image->encoding == "rgb8")
            {
                chanseq = MAPS_CHANNELSEQ_RGB;
            }
            else if (ros_image->encoding ==  "bgr8")
            {
                chanseq = MAPS_CHANNELSEQ_BGR;
            }
            else if (ros_image->encoding == "mono8")
            {
                chanseq = MAPS_CHANNELSEQ_GRAY;
            }
            else if (ros_image->encoding == "mono16") 
            {
                chanseq = MAPS_CHANNELSEQ_GRAY;
                depth = 16;
            } 
            else if (ros_image->encoding == "rgba8")
            {
                chanseq = MAPS_CHANNELSEQ_RGBA;
            }
            else if (ros_image->encoding == "bgra8")
            {
                chanseq = MAPS_CHANNELSEQ_BGRA;
            }
            else 
            {
                if (m_sub)
                {
                    m_sub.reset(); // stop topic subscription
                }
                MAPSStreamedString ss;
                ss << "Image format not supported: " << (const char *) ros_image->encoding.c_str();
                Error(ss);
                return;
            }
            IplImage model = MAPS::IplImageModel(ros_image->width, ros_image->height, chanseq, 0, depth);
            Output(0).AllocOutputBufferIplImage(model);
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

        IplImage& image_out = ioeltout->IplImage();
        int size = MIN((unsigned int)image_out.imageSize, ros_image->data.size());
	    MAPS::Memcpy(image_out.imageData, (char*)&ros_image->data[0], size);
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