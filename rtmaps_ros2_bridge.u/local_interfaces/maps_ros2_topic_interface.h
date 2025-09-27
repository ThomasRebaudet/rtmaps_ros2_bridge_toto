#pragma once

#include "maps.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

enum class Topic_Type
{
    String,
    Int32,
    Int32Array,
    Int64,
    Int64Array,
    Float32,
    Float32Array,
    Float64,
    Float64Array,
    Uint8Array,
};

enum class Endpoint_Type
{
    Subscriber,
    Publisher
};

class MAPS_ros2 : public MAPSComponent
{
public:
    friend class MAPSros2_topic_interface;
    MAPS_ros2(const char *componentName, MAPSComponentDefinition& md): MAPSComponent(componentName,md) {};
};

class MAPSros2_topic_interface
{
public:
    MAPSros2_topic_interface(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName)
    : m_component(component), m_node(node), m_endpointType(endpointType), m_topicName(topicName), m_inOutName(inOutName)
    {}

    virtual void Dynamic() = 0;
    virtual bool Birth() = 0;
    virtual void Core() {};
    virtual void Death() = 0;

    MAPSIOElt*      StartReading(MAPSInput& input);
    MAPSTimestamp   SynchroStartReading(int nb_inputs, MAPSInput** inputs, MAPSIOElt** ioelts);
    MAPSIOElt*      StartWriting(MAPSOutput& output);
    void            StopWriting(MAPSIOElt* IOElt, bool discard = false);
    MAPSInput&      Input(int index);
    MAPSInput&      Input(const char* name);
    MAPSOutput&     Output(int index);
    MAPSOutput&     Output(const char* name);

    MAPSInput&      NewInput(const char* model, const char* name = nullptr);
    MAPSOutput&     NewOutput(const char* model, const char* name = nullptr);
    MAPSProperty&   NewProperty(const char* model, const char* name = nullptr);

    void          ReportInfo(const char* message);
    void          ReportWarning(const char* message);
    void          ReportError(const char* message);
    void          Error(const char* message);

    MAPSString  GetStringProperty(const char* name);
    int64_t     GetIntegerProperty(const char* name);
    bool        GetBoolProperty(const char* name);
    MAPSFloat64 GetFloatProperty(const char* name);
    void        DirectSetProperty(const char* p, const MAPSEnumStruct& enumStruct);
    const char* Name() const;

    rclcpp::Time MAPSTimestampToROSTime(MAPSTimestamp t) {rclcpp::Time rostime(t/1000000, (t%1000000)*1000); return rostime;} // using ctor(seconds, nanoseconds)
    MAPSTimestamp ROSTimeToMAPSTimestamp(const rclcpp::Time& t) {MAPSTimestamp ts; ts = (MAPSInt64)(t.nanoseconds()/1000); return ts;}

private:
    MAPS_ros2* m_component;

protected:
    std::shared_ptr<rclcpp::Node>   m_node;
    Endpoint_Type                   m_endpointType;
    std::string                     m_topicName;
    std::string                     m_outputName;
    std::string                     m_inputName;
    std::string                     m_inOutName;
    std_msgs::msg::Header 	        m_header;
};