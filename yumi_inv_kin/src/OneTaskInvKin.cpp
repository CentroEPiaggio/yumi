#include <OneTaskInvKin.h>
OneTaskInvKin::OneTaskInvKin()
{
	std::string robot_description, root_name, tip_name;
	robot_description = n_.getNamespace() + "/robot_description";

	// xml_string = "";

	if (!n_.getParam("root_name", root_name))
    {
        ROS_ERROR_STREAM("OneTaskInvKin: No root name found on parameter server ("<<n_.getNamespace()<<"/root_name)");
    }

    if (!n_.getParam("tip_name", tip_name))
    {
        ROS_ERROR_STREAM("OneTaskInvKin: No tip name found on parameter server ("<<n_.getNamespace()<<"/tip_name)");
    }

    // Construct an URDF model from the xml string
    std::string xml_string;

    if (n_.hasParam(robot_description))
        n_.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
        n_.shutdown();
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
        n_.shutdown();
    }

    ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        n_.shutdown();
    }
    ROS_INFO("Successfully parsed urdf file");
    
    KDL::Tree kdl_tree_;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        n_.shutdown();
    }

    // Populate the KDL chain
    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
          ROS_ERROR_STREAM( "    "<<(*it).first);

    }

    ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());


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

