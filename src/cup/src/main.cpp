//
// Created by derk on 4-6-20.
//

#include <ros/ros.h>
#include <cup/cup.hpp>

#include <iostream>

int main(int argc, char** argv)
{
    try {
        if(argc > 4)
        {
            std::string node_name("cup");
            std::string id(argv[1]);
            ros::init(argc, argv, node_name + id);
            Robot robot("/gripper_left", "/gripper_right");
            World world("/odom",0, robot);
            Cup cup(node_name, static_cast<unsigned short>(std::stoi(id)), std::make_tuple(std::stod(argv[2]),
                                                                                           std::stod(argv[3]), std::stod(argv[4])), world);

            ros::Rate r(20);
            while (ros::ok())
            {
                cup.display();
                r.sleep();
            }
        }
        else
            throw std::runtime_error("Usage: cup_node id x y z");
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}