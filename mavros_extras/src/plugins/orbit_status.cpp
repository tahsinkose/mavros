#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/OrbitStatus.h>

namespace mavros {
namespace extra_plugins {
class OrbitStatusPlugin : public plugin::PluginBase {
public:
	OrbitStatusPlugin() : PluginBase(), orbit_status_nh("~orbit_status") {}
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		orbit_execution_status = orbit_status_nh.advertise<mavros_msgs::OrbitStatus>("execution", 10);
	}

	Subscriptions get_subscriptions() override {
		return {make_handler(&OrbitStatusPlugin::handle_orbit_execution_status)};
	}

private:
	ros::NodeHandle orbit_status_nh;

	ros::Publisher orbit_execution_status;

	Eigen::Vector3d earth_t_px4_ned(const double& x, const double& y, const double& z) {
		return Eigen::Vector3d(y, x, -z);
	}
	/**
	 * @brief Publish <a
	 * href="https://mavlink.io/en/messages/common.html#ORBIT_EXECUTION_STATUS">mavlink
	 * ORBIT_EXECUTION_STATUS message</a> into the  orbit_status/execution topic.
	 */
	void handle_orbit_execution_status(const mavlink::mavlink_message_t *msg,
		mavlink::common::msg::ORBIT_EXECUTION_STATUS &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::OrbitStatus>();
		ros_msg->header.stamp.fromNSec(mav_msg.time_usec * 1e3);
		ros_msg->radius = mav_msg.radius;
		if (mav_msg.frame == 1) {
			const Eigen::Vector3d earth_t_orbit_center = earth_t_px4_ned(mav_msg.x / 1e7, mav_msg.y / 1e7, mav_msg.z);
			ros_msg->header.frame_id = "earth";
			ros_msg->x = earth_t_orbit_center(0);
			ros_msg->y = earth_t_orbit_center(1);
			ros_msg->z = earth_t_orbit_center(2);
		} else {
			ros_msg->header.frame_id = "global";
			ros_msg->x = mav_msg.x;
			ros_msg->y = mav_msg.y;
			ros_msg->z = mav_msg.z;
		}
		
		ros_msg->state = mav_msg.state;
		ros_msg->progress = mav_msg.progress;
		orbit_execution_status.publish(ros_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OrbitStatusPlugin,
	mavros::plugin::PluginBase)