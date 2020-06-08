//
// Created by derk on 4-6-20.
//

#include <ros/ros.h>
#include <cup/cup.h>

#include <iostream>

int main(int argc, char** argv)
{
    try {
        if(argc > 4)
        {
            std::string id(argv[1]);
            ros::init(argc, argv, "cup" + id);
            Robot robot("/gripper_left", "/gripper_right");
            World world("/odom",0, robot);
            Cup cup(static_cast<unsigned short>(std::stoi(id)), std::make_tuple(std::stod(argv[2]),
                                                                                           std::stod(argv[3]), std::stod(argv[4])), world);

            ros::Rate r(20);
            while (ros::ok())
            {
                cup.display();
                robot.updateSensors();
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