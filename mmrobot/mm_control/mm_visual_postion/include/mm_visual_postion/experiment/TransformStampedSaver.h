#ifndef __TRANSFORM_STAMPED_SAVER_H__
#define __TRANSFORM_STAMPED_SAVER_H__
#include <geometry_msgs/TransformStamped.h>
class TransformStampedSaver{
public:
    double saved_transform[3];
    double saved_rotation[4];
    TransformStampedSaver(){};
    TransformStampedSaver(geometry_msgs::TransformStamped& transform_stamped){
        fromTransform(transform_stamped);
    }
    void fromTransform(geometry_msgs::TransformStamped& transform_stamped){
        saved_transform[0] = transform_stamped.transform.translation.x;
        saved_transform[1] = transform_stamped.transform.translation.y;
        saved_transform[2] = transform_stamped.transform.translation.z;
        saved_rotation[0] = transform_stamped.transform.rotation.x;
        saved_rotation[1] = transform_stamped.transform.rotation.y;
        saved_rotation[2] = transform_stamped.transform.rotation.z;
        saved_rotation[3] = transform_stamped.transform.rotation.w;
    }
    void toTransform(geometry_msgs::TransformStamped& transform_stamped){
        transform_stamped.transform.translation.x = saved_transform[0];
        transform_stamped.transform.translation.y = saved_transform[1];
        transform_stamped.transform.translation.z = saved_transform[2];
        transform_stamped.transform.rotation.x = saved_rotation[0];
        transform_stamped.transform.rotation.y = saved_rotation[1];
        transform_stamped.transform.rotation.z = saved_rotation[2];
        transform_stamped.transform.rotation.w = saved_rotation[3];
    }
};
#endif