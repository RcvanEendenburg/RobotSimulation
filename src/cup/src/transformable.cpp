//
// Created by derk on 8-6-20.
//

#include <cup/transformable.h>
#include <cup/transform_manager.h>

namespace Marker
{
Transformable::Transformable(const std::string& name, int type)
    : node_(), publisher_(node_.advertise<visualization_msgs::Marker>(name, 1)), attached_(false)
{
    const std::string &fixed_frame = TransformManager::get().getFixedFrame();
    marker_.ns = name;
    marker_.type = type;

    marker_.header.frame_id = fixed_frame;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.lifetime = ros::Duration();

    transform_.frame_id_ = fixed_frame;
    transform_.child_frame_id_ = marker_.ns;
}

void Transformable::setColor(double r, double g, double b)
{
    marker_.color.r = r;
    marker_.color.g = g;
    marker_.color.b = b;
    marker_.color.a = 1.0;
}

void Transformable::setScale(double x, double y, double z)
{
    marker_.scale.x = x;
    marker_.scale.y = y;
    marker_.scale.z = z;
}

void Transformable::setPosition(double x, double y, double z)
{
    marker_.pose.position.x = x;
    marker_.pose.position.y = y;
    marker_.pose.position.z = z;

    transform_.setOrigin({x, y, z});
}

void Transformable::setOrientation(double x, double y, double z, double w)
{
    marker_.pose.orientation.x = x;
    marker_.pose.orientation.y = y;
    marker_.pose.orientation.z = z;
    marker_.pose.orientation.w = w;

    transform_.setRotation({x, y, z, w});
}

const tf::Vector3 &Transformable::getWorldPosition() const
{
    const auto &listener = TransformManager::get().getListener();
    const std::string &fixed_frame = TransformManager::get().getFixedFrame();

    tf::StampedTransform world_cup_tf;
    listener.lookupTransform(fixed_frame, marker_.ns, ros::Time(0), world_cup_tf);
    return world_cup_tf.getOrigin();
}

tf::Quaternion Transformable::getWorldOrientation() const
{
    const auto &listener = TransformManager::get().getListener();
    const std::string &fixed_frame = TransformManager::get().getFixedFrame();

    tf::StampedTransform world_cup_tf;
    listener.lookupTransform(fixed_frame, marker_.ns, ros::Time(0), world_cup_tf);
    return world_cup_tf.getRotation();
}

const tf::Vector3 &Transformable::getPosition() const
{
    return transform_.getOrigin();
}

bool Transformable::available() const
{
    const auto &listener = TransformManager::get().getListener();
    const std::string &fixed_frame = TransformManager::get().getFixedFrame();
    return listener.canTransform(fixed_frame, marker_.ns, ros::Time(0));
}

void Transformable::sendUpdate()
{
    marker_.header.stamp = ros::Time::now();
    transform_.stamp_ = ros::Time::now();
    publisher_.publish(marker_);
    auto& broadcaster = TransformManager::get().getBroadcaster();
    broadcaster.sendTransform(transform_);
}

double Transformable::getZ() const
{
    return getPosition().z();
}

void Transformable::setZ(double z)
{
    auto position = getWorldPosition();
    setPosition(position.x(), position.y(), z);
}

void Transformable::setPosition(const tf::Vector3& position)
{
    marker_.pose.position.x = position.x();
    marker_.pose.position.y = position.y();
    marker_.pose.position.z = position.z();

    transform_.setOrigin(position);
}

void Transformable::setOrientation(const tf::Quaternion& quaternion)
{
    marker_.pose.orientation.x = quaternion.x();
    marker_.pose.orientation.y = quaternion.y();
    marker_.pose.orientation.z = quaternion.z();
    marker_.pose.orientation.w = quaternion.w();

    transform_.setRotation(quaternion);
}

void Transformable::attach(const std::string& frame)
{
    const auto &listener = TransformManager::get().getListener();

    tf::StampedTransform frame_tf;
    if(listener.canTransform(frame, marker_.ns, ros::Time(0)))
    {
        listener.lookupTransform(frame, marker_.ns, ros::Time(0), frame_tf);
        transform_.frame_id_ = frame;
        marker_.header.frame_id = frame;
        setOrientation(frame_tf.getRotation());
        setPosition(frame_tf.getOrigin());
        attached_ = true;
    }
}

void Transformable::attach(const std::string& frame, double x_offset, double y_offset, double z_offset)
{
    const auto &listener = TransformManager::get().getListener();

    tf::StampedTransform frame_tf;
    if(listener.canTransform(frame, frame, ros::Time(0)))
    {
        listener.lookupTransform(frame, frame, ros::Time(0), frame_tf);
        transform_.frame_id_ = frame;
        marker_.header.frame_id = frame;
        setOrientation(frame_tf.getRotation());
        auto position = frame_tf.getOrigin();
        setPosition(position.x() + x_offset, position.y() + y_offset, position.z() + z_offset);
        attached_ = true;
    }
}

void Transformable::detach()
{
    const auto &listener = TransformManager::get().getListener();
    const std::string &fixed_frame = TransformManager::get().getFixedFrame();

    tf::StampedTransform frame_tf;
    if(listener.canTransform(fixed_frame, marker_.ns, ros::Time(0)))
    {
        listener.lookupTransform(fixed_frame, marker_.ns, ros::Time(0), frame_tf);
        auto position = frame_tf.getOrigin();
        auto orientation = frame_tf.getRotation();

        transform_.frame_id_ = fixed_frame;
        marker_.header.frame_id = fixed_frame;
        attached_ = false;

        setPosition(position);
        setOrientation(orientation);
    }
}

bool Transformable::isAttached() const
{
    return attached_;
}


bool Transformable::collide(const Transformable& transformable) const
{
    const auto &listener = TransformManager::get().getListener();
    tf::StampedTransform tf;
    if(listener.canTransform(marker_.ns, transformable.marker_.ns, ros::Time(0)))
    {
        listener.lookupTransform(marker_.ns, transformable.marker_.ns, ros::Time(0), tf);

        double x = tf.getOrigin().x();
        double y = tf.getOrigin().y();
        double z = tf.getOrigin().z();

        double radius = marker_.scale.x;
        double height = marker_.scale.z;

        const double error_margin = 0.001;

        return(std::abs(y) <= (radius/2) + error_margin &&
            z > 0 && z <= radius + error_margin &&
            x > 0 && x <= (height/2) + error_margin);
    }
    return false;
}

}