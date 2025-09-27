#pragma once

#include "maps_ros2_topic_interface.h"
#include "visualization_msgs/msg/marker.hpp"


#define MARKER_OUTPUT_COUNT                 12

#define MARKER_OUTPUT_NB_ARROW              0
#define MARKER_OUTPUT_NB_CUBE               1
#define MARKER_OUTPUT_NB_SPHERE             2
#define MARKER_OUTPUT_NB_CYLINDER           3
#define MARKER_OUTPUT_NB_LINE_STRIP         4
#define MARKER_OUTPUT_NB_LINE_LIST          5
#define MARKER_OUTPUT_NB_CUBE_LIST          6
#define MARKER_OUTPUT_NB_SPHERE_LIST        7
#define MARKER_OUTPUT_NB_POINTS             8
#define MARKER_OUTPUT_NB_TEXT_VIEW_FACING   9
#define MARKER_OUTPUT_NB_MESH_RESOURCE      10
#define MARKER_OUTPUT_NB_TRIANGLE_LIST      11


#define MARKER_OUTPUT_SIZE_MAX_ARROW         512
#define MARKER_OUTPUT_SIZE_MAX_CUBE          512
#define MARKER_OUTPUT_SIZE_MAX_SPHERE        512
#define MARKER_OUTPUT_SIZE_MAX_CYLINDER      512
#define MARKER_OUTPUT_SIZE_MAX_LINE_STRIP    1024*3
#define MARKER_OUTPUT_SIZE_MAX_LINE_LIST     1024*3
#define MARKER_OUTPUT_SIZE_MAX_POINTS        1024
#define MARKER_OUTPUT_SIZE_MAX_TRIANGLE_LIST 512


// Accumulate markers by namespace and id
class MarkersCache {
public:
    MarkersCache() { }
    typedef visualization_msgs::msg::Marker::ConstSharedPtr MarkerConstPtr;

    int apply_action(const MarkerConstPtr marker) {
        int concerned_output = GetMarkerOutputByType(marker);
        switch (marker->action) {
            case visualization_msgs::msg::Marker::DELETEALL:
                for (auto& c : m_markers_cache)
                    c.clear();
                break;
            case visualization_msgs::msg::Marker::DELETE:
                remove(concerned_output, marker);
                break;
            case visualization_msgs::msg::Marker::ADD: {
                add(concerned_output, marker);
                break;
            }
            default:
                break;
        }
        return concerned_output;
    }
    void add(int concerned_output, const MarkerConstPtr& m) {
        auto& map = m_markers_cache[concerned_output];
        map[get_key(m)] = std::make_shared< visualization_msgs::msg::Marker >(*m);
    }
    void remove(int concerned_output, const MarkerConstPtr& m) {
        auto& map = m_markers_cache[concerned_output];
        map.erase(get_key(m));
    }
    std::string get_key(const MarkerConstPtr& m) {
        std::string ret = m->ns + "##" + std::to_string(m->id);
        return ret;
    }
    std::vector< MarkerConstPtr > get_markers_by_output(int concerned_output) {
        std::vector< MarkerConstPtr > ret;
        auto& map = m_markers_cache[concerned_output];
        for (auto& e : map) {
            ret.push_back(e.second);
        }
        return ret;
    }
    void clear() {
        for (auto& e : m_markers_cache)
            e.clear();
    }
    int GetMarkerOutputByType(const visualization_msgs::msg::Marker::ConstSharedPtr& marker) {
        // get concerned output
        int concerned_output = 0;
        switch (marker->type) {
            case visualization_msgs::msg::Marker::ARROW :
                concerned_output = MARKER_OUTPUT_NB_ARROW;
                break;
            case visualization_msgs::msg::Marker::CUBE :
                concerned_output = MARKER_OUTPUT_NB_CUBE;
                break;
            case visualization_msgs::msg::Marker::SPHERE :
                concerned_output = MARKER_OUTPUT_NB_SPHERE;
                break;
            case visualization_msgs::msg::Marker::CYLINDER :
                concerned_output = MARKER_OUTPUT_NB_CYLINDER;
                break;
            case visualization_msgs::msg::Marker::POINTS :
                concerned_output = MARKER_OUTPUT_NB_POINTS;
                break;
            case visualization_msgs::msg::Marker::LINE_STRIP :
                concerned_output = MARKER_OUTPUT_NB_LINE_STRIP;
                break;
            case visualization_msgs::msg::Marker::LINE_LIST :
                concerned_output = MARKER_OUTPUT_NB_LINE_LIST;
                break;
            case visualization_msgs::msg::Marker::SPHERE_LIST :
                concerned_output = MARKER_OUTPUT_NB_SPHERE_LIST;
                break;
            case visualization_msgs::msg::Marker::TEXT_VIEW_FACING :
                concerned_output = MARKER_OUTPUT_NB_TEXT_VIEW_FACING;
                break;
            case visualization_msgs::msg::Marker::MESH_RESOURCE :
                concerned_output = MARKER_OUTPUT_NB_MESH_RESOURCE;
                break;
            case visualization_msgs::msg::Marker::TRIANGLE_LIST :
                concerned_output = MARKER_OUTPUT_NB_TRIANGLE_LIST;
                break;
            default:
                break;
        }
        return concerned_output;
    }

private:
    std::map< std::string, visualization_msgs::msg::Marker::ConstSharedPtr > m_markers_cache[MARKER_OUTPUT_COUNT];
};


class MAPSros2_topic_marker : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_marker(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

protected:
    void TopicCallbackMarker(const visualization_msgs::msg::Marker::SharedPtr msg);
    void MarkersWriteByType(std::vector< visualization_msgs::msg::Marker::ConstSharedPtr >& markers, int32_t marker_type, int concerned_output, MAPSTimestamp marker_ts);
    void OutputMarkerOnlyFill(MAPSRealObject& obj, const visualization_msgs::msg::Marker::ConstSharedPtr& marker, int id=0);

private:
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr m_sub;

protected:
    MarkersCache m_markers_cache;
    std::string m_outputsNames[12];
    bool m_transferRosTimestamp;
};