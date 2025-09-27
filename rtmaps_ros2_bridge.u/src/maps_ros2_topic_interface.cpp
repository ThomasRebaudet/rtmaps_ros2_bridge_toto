#include "maps_ros2_topic_interface.h"

MAPSIOElt* MAPSros2_topic_interface::StartReading(MAPSInput& input)
{
    return m_component->StartReading(input);
}
MAPSTimestamp MAPSros2_topic_interface::SynchroStartReading(int nb_inputs, MAPSInput** inputs, MAPSIOElt** ioelts)
{
    return m_component->SynchroStartReading2(nb_inputs, inputs, ioelts);
}
MAPSIOElt* MAPSros2_topic_interface::StartWriting(MAPSOutput& output)
{
    return m_component->StartWriting(output);
}
void MAPSros2_topic_interface::StopWriting(MAPSIOElt* IOElt, bool discard)
{
    return m_component->StopWriting(IOElt, discard);
}
MAPSInput& MAPSros2_topic_interface::Input(int index)
{
    return m_component->Input(index);
}
MAPSInput& MAPSros2_topic_interface::Input(const char* name)
{
    return m_component->Input(name);
}
MAPSOutput& MAPSros2_topic_interface::Output(int index)
{
    return m_component->Output(index);
}
MAPSOutput& MAPSros2_topic_interface::Output(const char* name)
{
    return m_component->Output(name);
}
MAPSInput& MAPSros2_topic_interface::NewInput(const char* model, const char* name)
{
    return m_component->NewInput(model, name);
}
MAPSOutput& MAPSros2_topic_interface::NewOutput(const char* model, const char* name)
{
    return m_component->NewOutput(model, name);
}
MAPSProperty& MAPSros2_topic_interface::NewProperty(const char* model, const char* name)
{
    return m_component->NewProperty(model, name);
}
MAPSString MAPSros2_topic_interface::GetStringProperty(const char* name)
{
    return m_component->GetStringProperty(name);
}
MAPSInt64 MAPSros2_topic_interface::GetIntegerProperty(const char* name)
{
    return m_component->GetIntegerProperty(name);
}
bool MAPSros2_topic_interface::GetBoolProperty(const char* name)
{
    return m_component->GetBoolProperty(name);
}
MAPSFloat64 MAPSros2_topic_interface::GetFloatProperty(const char* name)
{
    return m_component->GetFloatProperty(name);
}
void MAPSros2_topic_interface::DirectSetProperty(const char* p, const MAPSEnumStruct& enumStruct)
{
    m_component->DirectSetProperty(p, enumStruct);
}
void MAPSros2_topic_interface::Error(const char* message)
{
    try 
    {
        //Calling Error instead of ReportError + CommitSuicide allows to set the error state on the component (red bar in the Studio)/
        //But that requires to catch the exception that this function triggers since it can be called from a ROS callback: it won't
        //be caught by ROS.
        m_component->Error(message);
    } 
    catch (int maps_exception)
    {}
    m_component->CommitSuicide();
}
void MAPSros2_topic_interface::ReportInfo(const char* message)
{
    m_component->ReportInfo(message);
}
void MAPSros2_topic_interface::ReportWarning(const char* message)
{
    m_component->ReportWarning(message);
}
void MAPSros2_topic_interface::ReportError(const char* message)
{
    m_component->ReportError(message);
}

const char* MAPSros2_topic_interface::Name() const
{
    return m_component->Name();
}