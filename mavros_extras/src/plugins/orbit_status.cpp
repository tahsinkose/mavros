#include <mavros/mavros_plugin.h>

#include <algorithm>

#include <eigen_conversions/eigen_msg.h>
#include <mavros_msgs/OrbitStatus.h>
#include <pluginlib/class_list_macros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mavros {
namespace extra_plugins {

namespace {
constexpr int kTotalNumOfProgressCubes = 100;
}
class OrbitStatusPlugin : public plugin::PluginBase {
public:
	OrbitStatusPlugin() : PluginBase(), orbit_status_nh("~orbit_status") {
		viz_tool_.reset(new rviz_visual_tools::RvizVisualTools("earth", "orbit_viz"));
		tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		progress_cube_published_.resize(kTotalNumOfProgressCubes, false);
	}
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

	rviz_visual_tools::RvizVisualToolsPtr viz_tool_;

	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	std::vector<bool> progress_cube_published_;

	Eigen::Translation3d earth_t_px4_ned(const double& x, const double& y, const double& z) {
		return Eigen::Translation3d(y, x, -z);
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
		ros_msg->state = mav_msg.state;
		ros_msg->progress = std::clamp(mav_msg.progress, 0.0f, 1.0f);
		if (mav_msg.frame == 1) {
			const Eigen::Translation3d map_t_orbit_center = earth_t_px4_ned(mav_msg.x / 1e7, mav_msg.y / 1e7, mav_msg.z);
			const auto whole_ns = orbit_status_nh.getNamespace();
			const auto robot_name = whole_ns.substr(1, whole_ns.find("/", 1));
			ros_msg->header.frame_id = robot_name + "map";
			ros_msg->x = map_t_orbit_center.x();
			ros_msg->y = map_t_orbit_center.y();
			ros_msg->z = map_t_orbit_center.z();
			
			geometry_msgs::TransformStamped earth_T_map = tf_buffer_->lookupTransform("earth", ros_msg->header.frame_id, ros::Time(0));
			Eigen::Affine3d earth_T_map_eigen;
			tf::transformMsgToEigen(earth_T_map.transform, earth_T_map_eigen);
			const Eigen::Affine3d earth_T_orbit_center_eigen = earth_T_map_eigen * map_t_orbit_center;
			geometry_msgs::Pose earth_T_progress_bar;
			tf::poseEigenToMsg(earth_T_orbit_center_eigen, earth_T_progress_bar);
			const double progress_percent = ros_msg->progress * 100.0;
			std::stringstream progress;
			progress.setf(std::ios::fixed);
    		progress.precision(2);
			progress << "Inspection progress: " << (progress_percent) << "%";
			earth_T_progress_bar.position.z += 2.0;
			viz_tool_->publishText(earth_T_progress_bar, progress.str(), rviz_visual_tools::colors::GREEN, rviz_visual_tools::scales::XXXLARGE);
			earth_T_progress_bar.position.z -= 0.2;
			Eigen::Affine3d earth_T_progress_bar_affine;
			tf::poseMsgToEigen(earth_T_progress_bar, earth_T_progress_bar_affine);
			Eigen::Isometry3d earth_T_progress_bar_iso;
			earth_T_progress_bar_iso.translation() = earth_T_progress_bar_affine.translation();
			earth_T_progress_bar_iso.linear() = earth_T_progress_bar_affine.rotation();
			const double progress_bar_width = 1.0; 
			viz_tool_->publishWireframeCuboid(earth_T_progress_bar_iso, 0.1, progress_bar_width, 0.1, rviz_visual_tools::colors::GREEN, "Progress Bar", 1);
			Eigen::Isometry3d earth_T_progress_bar_left_anchor;
			earth_T_progress_bar_left_anchor = earth_T_progress_bar_iso;
			earth_T_progress_bar_left_anchor.translation().y() -= (progress_bar_width / 2.0);
			for (size_t i = 0; i < std::floor(progress_percent); i++) {
				if (progress_cube_published_[i]) {
					continue;
				}
				Eigen::Isometry3d earth_T_progress_cube = earth_T_progress_bar_left_anchor;
				earth_T_progress_cube.translation().y() += i / (double)kTotalNumOfProgressCubes;
				viz_tool_->publishCuboid(earth_T_progress_cube, 0.1, 0.01, 0.1, rviz_visual_tools::colors::GREEN);
				progress_cube_published_[i] = true;
			}
			viz_tool_->trigger();
		} else {
			ros_msg->header.frame_id = "global";
			ros_msg->x = mav_msg.x;
			ros_msg->y = mav_msg.y;
			ros_msg->z = mav_msg.z;
		}
		
		if (ros_msg->state == mavros_msgs::OrbitStatus::ORBIT_EXECUTION_STATE_FINISHED) {
			viz_tool_->deleteAllMarkers();
			viz_tool_->trigger();
			progress_cube_published_.resize(kTotalNumOfProgressCubes, false);
		}
		orbit_execution_status.publish(ros_msg);
	}
};
}  // namespace extra_plugins
}  // namespace mavros
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OrbitStatusPlugin,
	mavros::plugin::PluginBase)