/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/map.h"
#include "rclcpp/rclcpp.hpp"

#include <memory>


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dlo::MapNode>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
