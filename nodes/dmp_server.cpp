#include "dmp/dmp.h"
using namespace dmp;

std::vector<DMPData> active_dmp_list;

bool selectCallback(SetFnApprox::Request &req,
					SetFnApprox::Response &res )
{
	enum functionApproxMode mode;

	switch(req.fn_type)
	{
		case 1:
			mode = functionApprox_linear;
			res.success = true;
			break;
		case 2:
			mode = functionApprox_fourier;
			res.success = true;
			break;
		case 3:
			mode = functionApprox_radial;
			res.success = true;
			break;
		default:
			res.success = false;
			break;
	}

	selectFunctionApprox(mode);
	return true;
}

bool lfdCallback(LearnDMPFromDemo::Request  &req,
			     LearnDMPFromDemo::Response &res )
{
	learnFromDemo(req.demo, req.k_gains, req.d_gains, req.num_bases, res.dmp_list);
	res.tau = req.demo.times[req.demo.times.size()-1];
	return true;
}

bool planCallback(GetDMPPlan::Request  &req,
			      GetDMPPlan::Response &res )
{
	generatePlan(active_dmp_list, req.x_0, req.x_dot_0, req.t_0, req.goal, req.goal_thresh,
			     req.seg_length, req.tau, req.dt, req.integrate_iter, res.plan, res.at_goal);
	return true;
}

bool activeCallback(SetActiveDMP::Request &req,
					SetActiveDMP::Response &res )
{
	active_dmp_list = req.dmp_list;
	res.success = true;
	return res.success;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dmp_server");
	ros::NodeHandle n;

	ros::ServiceServer service1 = n.advertiseService("learn_dmp_from_demo", lfdCallback);
	ros::ServiceServer service2 = n.advertiseService("get_dmp_plan", planCallback);
	ros::ServiceServer service3 = n.advertiseService("set_active_dmp", activeCallback);
	ros::ServiceServer service4 = n.advertiseService("select_function_approx", selectCallback);

	ROS_INFO("DMP services are now ready");
	ros::spin();

	return 0;
}
