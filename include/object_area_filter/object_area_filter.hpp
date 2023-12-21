// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#ifndef OBJECT_AREA_FILTER_HPP_
#define OBJECT_AREA_FILTER_HPP_

#include <point_in_polygon/point_in_polygon.hpp>

#include <vector>

#include <ros/ros.h>
#include <object_msgs/ObjectArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace object_area_filter
{

class ObjectAreaFilter
{
public:
    ObjectAreaFilter();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber objects_sub_;
    ros::Publisher objects_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool transform_objects_;
    bool include_in_area_;

    std::string areas_frame_;

    PointInPolygon point_in_polygon_;

    std::vector<PointInPolygon::Polygon> convert_xml_to_polygons(const XmlRpc::XmlRpcValue &xml_value);
    void objects_callback(const object_msgs::ObjectArray::ConstPtr& msg);
};

} // namespace object_area_filter

#endif // OBJECT_AREA_FILTER_HPP_

