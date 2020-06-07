#include <cup/world.hpp>

World::World(std::string fixed_frame, double ground_level, Robot& robot) : n_(), fixed_frame_(std::move(fixed_frame)),
ground_level_(ground_level), robot_(robot), listener_(n_), broadcaster_()
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

void World::createRobotSensors()
{
    robot_.createSensors();
}

void World::updateRobotSensors()
{
    robot_.updateSensors(listener_, broadcaster_);
}

void World::applyGravity(tf::StampedTransform& marker_transform, visualization_msgs::Marker& marker)
{
    tf::StampedTransform world_cup_tf;
    if(listener_.canTransform(fixed_frame_,marker.ns,ros::Time(0))) {
        listener_.lookupTransform(fixed_frame_, marker.ns, ros::Time(0), world_cup_tf);
        auto origin = marker_transform.getOrigin();

        if (origin.z() > ground_level_) {
            double new_z = origin.z() - 0.01;
            if (new_z < ground_level_) {
                marker_transform.setOrigin(tf::Vector3(origin.x(), origin.y(), ground_level_));
                marker_transform.setRotation(tf::createQuaternionFromYaw(0));
            }
            else {
                marker_transform.setOrigin(tf::Vector3(origin.x(), origin.y(), new_z));
                marker.pose.position.z = new_z;
            }
        }
        else {
            marker.header.frame_id = fixed_frame_;
            marker.pose.orientation.x = world_cup_tf.getRotation().x();
            marker.pose.orientation.y = world_cup_tf.getRotation().y();
            marker.pose.orientation.z = world_cup_tf.getRotation().z();
            marker.pose.orientation.w = world_cup_tf.getRotation().w();

            marker.pose.position.x = world_cup_tf.getOrigin().x();
            marker.pose.position.y = world_cup_tf.getOrigin().y();
            marker.pose.position.z = world_cup_tf.getOrigin().z();

            marker_transform.frame_id_ = fixed_frame_;
            marker_transform.setRotation(world_cup_tf.getRotation());
            marker_transform.setOrigin(world_cup_tf.getOrigin());
        }
    }
    marker.header.stamp = ros::Time::now();
    marker_transform.stamp_ = ros::Time::now();
    broadcaster_.sendTransform(marker_transform);
}

bool World::robotGrabbedMarker(const visualization_msgs::Marker& marker) const
{
    return robot_.grabbedMarker(listener_, marker);
}

void World::followRobot(tf::StampedTransform& marker_transform, visualization_msgs::Marker& marker)
{
    robot_.followGripper(listener_, marker_transform, marker);
    marker.header.stamp = ros::Time::now();
    marker_transform.stamp_ = ros::Time::now();
    broadcaster_.sendTransform(marker_transform);
}

tf::Vector3 World::getMarkerPosition(const visualization_msgs::Marker& marker) const
{
    if(listener_.canTransform(fixed_frame_, marker.ns, ros::Time(0))) {
        tf::StampedTransform world_cup_tf;
        listener_.lookupTransform(fixed_frame_, marker.ns, ros::Time(0), world_cup_tf);
        return world_cup_tf.getOrigin();
    }
    return {};
}

tf::Quaternion World::getMarkerOrientation(const visualization_msgs::Marker& marker) const
{
    if(listener_.canTransform(fixed_frame_, marker.ns, ros::Time(0))) {
        tf::StampedTransform world_cup_tf;
        listener_.lookupTransform(fixed_frame_, marker.ns, ros::Time(0), world_cup_tf);
        return world_cup_tf.getRotation();
    }
    return {};
}