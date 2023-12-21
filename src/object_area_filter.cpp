// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <object_area_filter/object_area_filter.hpp>
#include <point_in_polygon/point_in_polygon.hpp>

#include <string>

#include <ros/ros.h>
#include <object_msgs/ObjectArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace object_area_filter
{

ObjectAreaFilter::ObjectAreaFilter() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
    // Get parameters
    std::string objects_in_topic, objects_out_topic;
    XmlRpc::XmlRpcValue xml_value;
    pnh_.param<std::string>("objects_in", objects_in_topic, "/objects_in");
    pnh_.param<std::string>("objects_out", objects_out_topic, "/object_array");
    pnh_.param<std::string>("areas_frame", areas_frame_, "map");
    pnh_.param<bool>("transform_objects", transform_objects_, false);
    pnh_.param<bool>("include_in_area", include_in_area_, true);
    pnh_.getParam("areas", xml_value);

    // Convert XmlRpcValue to polygons
    std::vector<PointInPolygon::Polygon> polygons = convert_xml_to_polygons(xml_value);

    // Set polygons
    point_in_polygon_.set_polygons(polygons);

    // Prepare subscriber and publisher
    objects_sub_ = nh_.subscribe(objects_in_topic, 1, &ObjectAreaFilter::objects_callback, this);
    objects_pub_ = nh_.advertise<object_msgs::ObjectArray>(objects_out_topic, 1);
}

std::vector<PointInPolygon::Polygon> ObjectAreaFilter::convert_xml_to_polygons(const XmlRpc::XmlRpcValue &xml_value)
{
    if (xml_value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Invalid area vector.");
        ros::shutdown();
    }

    if (xml_value.size() == 0)
        ROS_WARN("No areas found. All objects will be considered to be out of area.");

    std::vector<PointInPolygon::Polygon> polygons;
    for (int i = 0; i < xml_value.size(); i++)
    {
        if (xml_value[i].getType() != XmlRpc::XmlRpcValue::TypeArray
            || xml_value[i].size() == 0)
        {
            ROS_ERROR("Invalid area found.");
            ros::shutdown();
        }

        PointInPolygon::Polygon polygon;
        for (int j = 0; j < xml_value[i].size(); j++)
        {
            if (xml_value[i][j].getType() != XmlRpc::XmlRpcValue::TypeArray
                || xml_value[i][j].size() != 2)
            {
                ROS_ERROR("Invalid area point found.");
                ros::shutdown();
            }

            PointInPolygon::Point point;
            point.x = static_cast<double>(xml_value[i][j][0]);
            point.y = static_cast<double>(xml_value[i][j][1]);

            polygon.points.push_back(point);
        }

        polygons.push_back(polygon);
    }

    return polygons;
}

void ObjectAreaFilter::objects_callback(const object_msgs::ObjectArray::ConstPtr& msg)
{
    // Get transform from objects frame to areas frame.
    geometry_msgs::TransformStamped objects2areas;
    try
    {
        objects2areas = tf_buffer_.lookupTransform(
            areas_frame_,
            msg->header.frame_id,
            ros::Time(0)
        );
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_DELAYED_THROTTLE(5, "%s", ex.what());
        return;
    }

    object_msgs::ObjectArray object_array, object_array_filtered;
    object_array = *msg;

    // Transform object positions to areas frame and filter objects.
    for (size_t i = 0; i < object_array.objects.size(); i++)
    {
        geometry_msgs::Point objects_point, areas_point;
        objects_point.x = object_array.objects[i].pose.position.x;
        objects_point.y = object_array.objects[i].pose.position.y;
        objects_point.z = object_array.objects[i].pose.position.z;
        tf2::doTransform(objects_point, areas_point, objects2areas);

        if (transform_objects_)
        {
            object_array.objects[i].pose.position.x = areas_point.x;
            object_array.objects[i].pose.position.y = areas_point.y;
            object_array.objects[i].pose.position.z = areas_point.z;
        }

        bool is_inside = point_in_polygon_.is_inside(areas_point.x, areas_point.y);
        if (is_inside == include_in_area_)
            object_array_filtered.objects.push_back(object_array.objects[i]);
    }

    // Prepare header and overwrite frame if necessary.
    object_array_filtered.header = msg->header;

    if (transform_objects_)
        object_array_filtered.header.frame_id = areas_frame_;

    objects_pub_.publish(object_array_filtered);
}

} // namespace object_area_filter
