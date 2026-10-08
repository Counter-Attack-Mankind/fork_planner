#include "forklift_map/path_debug_visualizer.h"

#include <cmath>
#include <sstream>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace forklift_map {
namespace path_debug {
namespace {

std_msgs::ColorRGBA rgba(float r, float g, float b, float a) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

geometry_msgs::Point point(double x, double y, double z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

visualization_msgs::Marker baseMarker(const std::string& frame,
                                      const std::string& ns,
                                      int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = ros::Time(0);
    m.ns = ns;
    m.id = id;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
}

const char* layerName(DebugPathLayerType type) {
    switch (type) {
        case DebugPathLayerType::SKELETON: return "skeleton";
        case DebugPathLayerType::CLOTHOID: return "clothoid";
        case DebugPathLayerType::LANE_SHIFT: return "lane_shift";
        case DebugPathLayerType::ARC_FALLBACK: return "arc_fallback";
    }
    return "unknown";
}

std_msgs::ColorRGBA layerColor(DebugPathLayerType type) {
    switch (type) {
        case DebugPathLayerType::SKELETON:
            return rgba(1.0f, 1.0f, 1.0f, 0.95f);
        case DebugPathLayerType::CLOTHOID:
            return rgba(0.10f, 0.45f, 1.0f, 0.95f);
        case DebugPathLayerType::LANE_SHIFT:
            return rgba(0.0f, 0.95f, 1.0f, 0.95f);
        case DebugPathLayerType::ARC_FALLBACK:
            return rgba(1.0f, 0.08f, 0.08f, 0.95f);
    }
    return rgba(1.0f, 1.0f, 1.0f, 0.8f);
}

double layerWidth(DebugPathLayerType type) {
    switch (type) {
        case DebugPathLayerType::SKELETON: return 0.018;
        case DebugPathLayerType::CLOTHOID: return 0.026;
        case DebugPathLayerType::LANE_SHIFT: return 0.026;
        case DebugPathLayerType::ARC_FALLBACK: return 0.030;
    }
    return 0.018;
}

void addText(visualization_msgs::MarkerArray& arr,
             const std::string& frame,
             const std::string& ns,
             int id,
             double x,
             double y,
             double z,
             double height,
             const std::string& text,
             const std_msgs::ColorRGBA& color) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.scale.z = height;
    m.text = text;
    m.color = color;
    arr.markers.push_back(m);
}

void addSphere(visualization_msgs::MarkerArray& arr,
               const std::string& frame,
               const std::string& ns,
               int id,
               double x,
               double y,
               double z,
               double radius,
               const std_msgs::ColorRGBA& color) {
    auto m = baseMarker(frame, ns, id);
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.scale.x = radius;
    m.scale.y = radius;
    m.scale.z = radius * 0.45;
    m.color = color;
    arr.markers.push_back(m);
}

void addLayerLine(visualization_msgs::MarkerArray& arr,
                  int& id,
                  const std::string& frame,
                  const std::string& ns,
                  const DebugPathLayer& layer,
                  double z_offset) {
    if (layer.points.size() < 2) return;
    auto m = baseMarker(frame, ns, id++);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = layerWidth(layer.type);
    m.color = layerColor(layer.type);
    m.points.reserve(layer.points.size());
    for (const DebugPathPoint& p : layer.points) {
        m.points.push_back(point(p.x, p.y, z_offset));
    }
    arr.markers.push_back(m);
}

void addReverseSegments(visualization_msgs::MarkerArray& arr,
                        int& id,
                        const std::string& frame,
                        const std::string& ns,
                        const RoughPath& path,
                        double z_offset) {
    if (path.size() < 2) return;
    size_t i = 1;
    while (i < path.size()) {
        while (i < path.size() && path[i].type != WpType::REVERSE) ++i;
        if (i >= path.size()) break;
        auto m = baseMarker(frame, ns, id++);
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.scale.x = 0.034;
        m.color = rgba(1.0f, 0.55f, 0.0f, 0.95f);
        m.points.push_back(point(path[i - 1].x, path[i - 1].y, z_offset));
        while (i < path.size() && path[i].type == WpType::REVERSE) {
            m.points.push_back(point(path[i].x, path[i].y, z_offset));
            ++i;
        }
        if (m.points.size() >= 2) arr.markers.push_back(m);
    }
}

}  // namespace

void addPathDebugLayers(visualization_msgs::MarkerArray& arr,
                        int& id,
                        const std::string& frame_id,
                        const std::string& ns_prefix,
                        const RoughPath& path,
                        const PathGenerationInfo& info,
                        const std::string& label,
                        const PathDebugVisualOptions& options) {
    if (!options.show_layers) return;

    const double base_z = options.z_offset;
    const std::string ns = ns_prefix + "_debug_layers";

    if (info.debug_layers.empty()) {
        if (!path.empty()) {
            addText(arr, frame_id, ns + "_empty", id++, path.front().x,
                    path.front().y, base_z + 0.08, 0.055,
                    label + " no debug_layers",
                    rgba(1.0f, 0.9f, 0.2f, 1.0f));
        }
        return;
    }

    for (const DebugPathLayer& layer : info.debug_layers) {
        addLayerLine(arr, id, frame_id,
                     ns + "_" + layerName(layer.type), layer, base_z);
        if (options.show_corner_labels &&
            layer.type == DebugPathLayerType::SKELETON) {
            for (size_t i = 0; i < layer.points.size(); ++i) {
                const DebugPathPoint& p = layer.points[i];
                addSphere(arr, frame_id, ns + "_skeleton_point", id++,
                          p.x, p.y, base_z + 0.012, 0.055,
                          rgba(1.0f, 1.0f, 1.0f, 0.9f));
                std::ostringstream ss;
                ss << "S" << i;
                addText(arr, frame_id, ns + "_skeleton_label", id++,
                        p.x, p.y, base_z + 0.07, 0.045, ss.str(),
                        rgba(1.0f, 1.0f, 1.0f, 0.95f));
            }
        }
    }

    if (options.show_turn_types) {
        addReverseSegments(arr, id, frame_id, ns + "_reverse", path,
                           base_z + 0.025);
    }

    if (!path.empty()) {
        addText(arr, frame_id, ns + "_legend", id++, path.front().x,
                path.front().y, base_z + 0.13, 0.052,
                "white=skeleton blue=clothoid cyan=lane_shift red=arc orange=reverse",
                rgba(1.0f, 1.0f, 1.0f, 0.95f));
    }
}

}  // namespace path_debug
}  // namespace forklift_map
