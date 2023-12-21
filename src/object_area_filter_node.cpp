// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <object_area_filter/object_area_filter.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_area_filter");
    object_area_filter::ObjectAreaFilter object_area_filter;
    ros::spin();

    return 0;
}
