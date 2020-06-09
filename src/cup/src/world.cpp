#include <cup/world.h>
#include <cup/transform_manager.h>

World::World(std::string fixed_frame, double ground_level, Robot &robot)
    : n_(), fixed_frame_(std::move(fixed_frame)),
      ground_level_(ground_level), robot_(robot), listener_(n_), broadcaster_()
{
    TransformManager::get().setFixedFrame(fixed_frame_);
}

double World::getGroundLevel() const
{
    return ground_level_;
}

void World::applyGravity(Marker::Transformable &marker)
{
    if (marker.available())
    {
        if (marker.isAttached())
        {
            static int attach_counter = 0;
            attach_counter++;

            if (attach_counter > 5)
            {
                marker.detach();
                marker.sendUpdate();
                attach_counter = 0;
            }
        }
        else
        {
            double z = marker.getZ();

            if (z > ground_level_)
            {
                double new_z = z - 0.01;
                if (new_z < ground_level_)
                {
                    marker.setZ(ground_level_);
                    marker.setOrientation(tf::createQuaternionFromYaw(0));
                }
                else
                {
                    marker.setZ(new_z);
                }
            }
        }
        marker.sendUpdate();
    }
}

bool World::robotGrabbedMarker(const Marker::Transformable &marker) const
{
    return robot_.grabbedMarker(marker);
}

bool World::robotPushedMarkerLeft(const Marker::Transformable& marker) const
{
    return robot_.pushedMarkerLeft(marker);
}

bool World::robotPushedMarkerRight(const Marker::Transformable& marker) const
{
    return robot_.pushedMarkerRight(marker);
}

void World::robotPushLeft(Marker::Transformable &marker)
{
    robot_.pushLeft(marker);
    marker.sendUpdate();
}

void World::robotPushRight(Marker::Transformable &marker)
{
    robot_.pushRight(marker);
    marker.sendUpdate();
}

void World::followRobot(Marker::Transformable &marker)
{
    robot_.followGripper(marker);
    marker.sendUpdate();
}