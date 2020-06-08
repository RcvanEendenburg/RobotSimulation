//
// Created by derk on 8-6-20.
//

#ifndef TRANSFORM_MANAGER_H
#define TRANSFORM_MANAGER_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class TransformManager
{
public:
    /**
     * Gets the transform manager.
     * @return The transform manager.
     */
    static TransformManager& get()
    {
        static TransformManager transformManager;
        return transformManager;
    }

    /**
     * Registers the fixed frame of the world.
     * @param fixed_frame The fixed frame.
     */
    void setFixedFrame(const std::string& fixed_frame) {fixed_frame_ = fixed_frame;}

    /**
     * Retrieves the fixed frame of the world.
     * @return The fixed frame.
     */
    const std::string& getFixedFrame() {return fixed_frame_;}

    /**
     * Gets the listener.
     * @return The listener.
     */
    const tf::TransformListener& getListener() const {return listener_;}

    /**
     * Gets the broadcaster.
     * @return The broadcaster
     */
    tf::TransformBroadcaster& getBroadcaster() {return broadcaster_;}

private:
    TransformManager() : listener_(node_), broadcaster_(), fixed_frame_() {}
    ~TransformManager() = default;

    ros::NodeHandle node_;
    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;
    std::string fixed_frame_;
};

#endif //TRANSFORM_MANAGER_H
