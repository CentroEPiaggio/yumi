#include <OneTaskInvKin.h>
OneTaskInvKin::OneTaskInvKin()
{
	std::string root_name, tip_name;

	if (!n_.getParam("root_name", root_name))
    {
        ROS_ERROR_STREAM("OneTaskInvKin: No root name found on parameter server ("<<n_.getNamespace()<<"/root_name)");
    }

    if (!n_.getParam("tip_name", tip_name))
    {
        ROS_ERROR_STREAM("OneTaskInvKin: No tip name found on parameter server ("<<n_.getNamespace()<<"/tip_name)");
    }


	// Eigen::Vector3d vec_cam_2_base;
	// Eigen::Quaterniond quat_cam_2_base;
	
	// vec_cam_2_base << translation[0], translation[1], translation[2];
	// quat_cam_2_base.w() = rotation[3];
	// quat_cam_2_base.vec() << rotation[0], rotation[1], rotation[2];

	// //Subscriber
	// sub_button_start_teach_ = n_.subscribe(topic_botton_start_teach_, 1, &OneTaskInvKin::callback_button_start_teach, this);

	// //Publisher
 //    pub_pos_des_ = n_.advertise<geometry_msgs::PoseStamped>(topic_pose_des, 1);

}

OneTaskInvKin::~OneTaskInvKin()
{

}

