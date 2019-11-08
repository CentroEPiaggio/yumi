#include <control_msgs/JointControllerState.h> 

#include <urdf/model.h>
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen3/Eigen/Eigen>
#include <skew_symmetric.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>


class OneTaskInvKin
{
	public:
		OneTaskInvKin();
		~OneTaskInvKin();

		void run();


  	double dt_;

	private:

		void callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg);
		void callback_des_pose(const geometry_msgs::Pose::ConstPtr& msg);


		void init();
		void update();


	
		ros::NodeHandle n_;
		ros::Subscriber sub_joint_states_, sub_des_pos_;

		KDL::Chain kdl_chain_;

		KDL::JntArray  q_msr_;          // Joint measured positions
		KDL::JntArray  q_;        		// Joint computed positions
		KDL::Jacobian  J_;            // Jacobian
		KDL::Frame     x_;            // Tip pose                                                                                                                                       

		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

		Eigen::VectorXd joint_location_;
		Eigen::Vector3d pos_d_;
		Eigen::Quaterniond quat_d_;
		Eigen::MatrixXd k_;
		bool flag_joint_msr_;
};