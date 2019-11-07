#include <control_msgs/JointControllerState.h> // TODO: state message for all controllers?

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


class OneTaskInvKin
{
	public:
		OneTaskInvKin();
		~OneTaskInvKin();

  	double dt_;

	private:
		ros::NodeHandle n_;
		KDL::Chain kdl_chain_;


};