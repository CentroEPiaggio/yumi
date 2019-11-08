#include <OneTaskInvKin.h>
OneTaskInvKin::OneTaskInvKin()
{

	std::string robot_description, root_name, tip_name;
	robot_description = n_.getNamespace() + "/robot_description";


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

    std::vector<double> joint_location;
    std::string topic_joint_cmd;

    n_.getParam("joint_location", joint_location);
    n_.getParam("topic_joint_cmd", topic_joint_cmd);

    joint_location_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
    joint_location_ << joint_location[0], joint_location[1], joint_location[2],joint_location[3], joint_location[4], joint_location[5], joint_location[6]; 

	jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

	q_msr_.resize(kdl_chain_.getNrOfJoints());
	q_.resize(kdl_chain_.getNrOfJoints());
	J_.resize(kdl_chain_.getNrOfJoints());
	q_eig_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());

	k_ = Eigen::Matrix<double, 6, 6>::Identity() * 5.0;
	step_ = 2;
	flag_joint_msr_ = false;

	//Subscriber
	sub_joint_states_ = n_.subscribe("/yumi/joint_states", 1, &OneTaskInvKin::callback_joint_states, this);
	sub_des_pos_ = n_.subscribe("command", 10, &OneTaskInvKin::callback_des_pose, this);

	//Publisher
    pub_joint_cmd_ = n_.advertise<std_msgs::Float64MultiArray>(topic_joint_cmd, 1);

}

OneTaskInvKin::~OneTaskInvKin()
{

}

void OneTaskInvKin::callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
	{
		q_msr_(i) = msg->position[joint_location_(i)];
	}
	if(flag_joint_msr_ == false)
	{
		step_ = 0;
		flag_joint_msr_ = true;
	}
	
}

void OneTaskInvKin::callback_des_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
	pos_d_(0) = msg->position.x;
	pos_d_(1) = msg->position.y;
	pos_d_(2) = msg->position.z;

	quat_d_.w() = msg->orientation.w;
	quat_d_.x() = msg->orientation.x;
	quat_d_.y() = msg->orientation.y;
	quat_d_.z() = msg->orientation.z;
}


void OneTaskInvKin::init()
{
	q_ = q_msr_;

	// test
	q_msr_(0) = q_(0);
	q_msr_(1) = q_(1);
	q_msr_(2) = q_(6);
	q_msr_(3) = q_(2);
	q_msr_(4) = q_(3);
	q_msr_(5) = q_(4);
	q_msr_(6) = q_(5);
	// test

	for(int i = 0; i < kdl_chain_.getNrOfJoints(); i++) q_eig_(i) = q_msr_(i);

	// computing forward kinematics
   	fk_pos_solver_->JntToCart(q_msr_, x_);
   	Eigen::Matrix3d orient_d;
   	for(int i = 0; i < 3; i++)
	{
		pos_d_(i) = x_.p(i);

		for(int j = 0; j < 3; j++)
		{
			orient_d(i, j) = x_.M(i, j);
		}
	}
	quat_d_ = orient_d;

	// std::cout<<"q_msr_: "<< q_msr_(0)<<", "<< q_msr_(1)<<", "<< q_msr_(2)<< ", "<< q_msr_(3)<<", "<< q_msr_(4)<<", "<< q_msr_(5)<<", "<< q_msr_(6)<<std::endl;
	std::cout<<"pos_init: "<< x_.p(0)<<", "<< x_.p(1)<<", "<< x_.p(2)<<std::endl;
	// std::cout<<"pos_init: "<< pos_d_.x()<<", "<< pos_d_.y()<<", "<< pos_d_.z()<<std::endl;
	std::cout<<"quat_init: "<< quat_d_.w()<<", "<< quat_d_.x()<<", "<< quat_d_.y()<<", "<< quat_d_.z()<<std::endl;
	// getchar();
}

void OneTaskInvKin::update()
{
	// Compute the forward kinematics and Jacobian (at this location).
	fk_pos_solver_->JntToCart(q_, x_);
	jnt_to_jac_solver_->JntToJac(q_, J_);

	// Position and rot matrix from kdl to Eigen
	Eigen::Vector3d pos, e_pos, e_quat;
	Eigen::Matrix3d orient, skew;
	Eigen::MatrixXd Jac_, Jac_pinv;
	Eigen::Quaterniond quat;
	Eigen::VectorXd e(Eigen::VectorXd::Zero(6));
	Eigen::VectorXd qdot(Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints()));
	KDL::Vector quat_d_vec;
	std_msgs::Float64MultiArray joint_cmd;
	joint_cmd.data.clear();

	Jac_.resize(6, kdl_chain_.getNrOfJoints());
	Jac_pinv.resize(6, kdl_chain_.getNrOfJoints());

	for(int i = 0; i < 3; i++)
	{
		pos(i) = x_.p(i);

		for(int j = 0; j < 3; j++)
		{
			orient(i, j) = x_.M(i, j);
		}
	}

	for(int i = 0; i < 6 ; i++)
	{
		for(int j = 0; j < kdl_chain_.getNrOfJoints(); j++)
		{
			Jac_(i, j) = J_(i, j);

		}	
	}

	//from matrix to quat
	quat = orient;
	quat.normalize();

	quat_d_vec(0) = quat_d_.x();
	quat_d_vec(1) = quat_d_.y();
	quat_d_vec(2) = quat_d_.z();

	skew_symmetric(quat_d_vec, skew);

	e_pos = pos_d_ - pos;
	e_quat = (quat.w() * quat_d_.vec()) - (quat_d_.w() * quat.vec()) - (skew * quat.vec()); 

	e << e_pos, e_quat;

	pseudo_inverse(Jac_, Jac_pinv, true);
		
	qdot = Jac_pinv * (k_ * e);

	q_eig_ += qdot * dt_;

	for(int i = 0; i < kdl_chain_.getNrOfJoints(); i++) 
	{
		q_(i) = q_eig_(i);	
	}

	joint_cmd.data.push_back(q_(0));
	joint_cmd.data.push_back(q_(1));
	joint_cmd.data.push_back(q_(6));
	joint_cmd.data.push_back(q_(2));
	joint_cmd.data.push_back(q_(3));
	joint_cmd.data.push_back(q_(4));
	joint_cmd.data.push_back(q_(5));

	// pub_joint_cmd_.publish(joint_cmd);

}

void OneTaskInvKin::run()
{
	switch(step_)
	{
		case 0:
		{
			init();
			step_ = 1;
			break;
		}
		case 1:
		{
			update();
			break;
		}
		case 2:
		{
			// nothing
			break;
		}
	}
}