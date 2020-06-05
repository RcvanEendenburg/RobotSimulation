#include <cup/world.hpp>

World::World(std::string fixed_frame, double ground_level, Robot& robot) : fixed_frame_(std::move(fixed_frame)),
ground_level_(ground_level), robot_(robot)
{

}

void World::createMarker(visualization_msgs::Marker& marker, tf::StampedTransform& transform,
    std::tuple<double,double,double> position) const
{
    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.pose.position.x = std::get<0>(position);
    marker.pose.position.y = std::get<1>(position);
    marker.pose.position.z = std::get<2>(position) + ground_level_;

    transform.frame_id_ = fixed_frame_;
    transform.child_frame_id_ = marker.ns;
    transform.stamp_ = ros::Time::now();
    transform.setOrigin(tf::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
    transform.setRotation(tf::createQuaternionFromYaw(0));
}

void World::applyGravity(tf::TransformListener& listener, tf::StampedTransform& transform,
    visualization_msgs::Marker& marker)
{
    if (transform.getOrigin().z() <= ground_level_)
    {
        auto origin = transform.getOrigin();
        transform.setOrigin(tf::Vector3(origin.x(), origin.y(), ground_level_));
        marker.pose.position.z = ground_level_;
        marker.pose.orientation.z = 0;
    }
    else
    {
        auto origin = transform.getOrigin();
        double new_z = transform.getOrigin().z() - 0.01;
        transform.setOrigin(tf::Vector3(origin.x(), origin.y(), new_z));
        marker.header.frame_id = fixed_frame_;
        marker.pose.orientation.x = world_transform_.getRotation().x();
        marker.pose.orientation.y = world_transform_.getRotation().y();
        marker.pose.orientation.z = new_z;
        marker.pose.orientation.w = world_transform_.getRotation().w();
        marker.pose.position.x = world_transform_.getOrigin().x();
        marker.pose.position.y = world_transform_.getOrigin().y();
        marker.pose.position.z = world_transform_.getOrigin().z();

        transform.frame_id_ = fixed_frame_;
        transform.setRotation(world_transform_.getRotation());
        transform.setOrigin(world_transform_.getOrigin());
    }
}

const std::string& World::getFixedFrame() const
{
    return fixed_frame_;
}

bool World::robotGrabbedMarker(tf::TransformListener& listener, const visualization_msgs::Marker& marker) const
{
    return robot_.grabbedMarker(listener, marker);
}

void World::followRobot(tf::TransformListener& listener, tf::StampedTransform& transform,
    visualization_msgs::Marker& marker) const
{
    robot_.followGripper(listener, transform, marker);
}

bool World::setTransforms(tf::TransformListener& listener, visualization_msgs::Marker& marker)
{
    if(listener.canTransform(fixed_frame_,marker.ns,ros::Time(0)))
    {
        listener.lookupTransform(fixed_frame_, marker.ns, ros::Time(0), world_transform_);
        return robot_.setTransforms(listener, marker);
    }
    return false;
}

const tf::Vector3& World::getMarkerPosition(const visualization_msgs::Marker& marker) const
{
    return world_transform_.getOrigin();
}

tf::Quaternion World::getMarkerOrientation(const visualization_msgs::Marker& marker) const
{
    return world_transform_.getRotation();
}