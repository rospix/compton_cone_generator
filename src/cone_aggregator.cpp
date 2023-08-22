/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <rad_msgs/Cone.h>

#include <Eigen/Eigen>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/batch_visualizer.h>

//}

namespace compton_cone_generator
{

/* ConeAggregator //{ */

class ConeAggregator : public nodelet::Nodelet {

public:
  virtual void onInit();

  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ----------------------- subscribers ---------------------- |

  std::vector<mrs_lib::SubscribeHandler<rad_msgs::Cone>> sh_cones_;

  void callbackCone(const rad_msgs::Cone::ConstPtr sh_ptr);
  void callbackTimeout(const std::string &topic_name, const ros::Time &last_msg_time);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<rad_msgs::Cone> ph_cones_;

  // | -------------------- the transformer  -------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | -------------------- batch visualizer -------------------- |

  mrs_lib::BatchVisualizer batch_vizualizer_;

  // | ------------------------- params ------------------------- |

  std::string              _uav_name_;
  std::vector<std::string> _uav_names_;
  std::string              _topic_name_;
  std::string              _target_frame_;
};
//}

//}

/* onInit() //{ */

void ConeAggregator::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ConeAggregator]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ConeAggregator");

  param_loader.loadParam("network/robot_names", _uav_names_);
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("topic_name", _topic_name_);
  param_loader.loadParam("target_frame", _target_frame_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ConeAggregator]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  ph_cones_ = mrs_lib::PublisherHandler<rad_msgs::Cone>(nh_, "cones_out", 10);

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "ConeAggregator");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | -------------------- batch visualizer -------------------- |

  batch_vizualizer_ = mrs_lib::BatchVisualizer(nh_, "compton_cones", _target_frame_);

  batch_vizualizer_.clearBuffers();
  batch_vizualizer_.clearVisuals();

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ConeAggregator";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();
  shopts.no_message_timeout = ros::Duration(10.0);

  for (int i = 0; i < int(_uav_names_.size()); i++) {

    std::string topic_name = std::string("/") + _uav_names_[i] + std::string("/") + _topic_name_;

    ROS_INFO("[MpcTracker]: subscribing to %s", topic_name.c_str());

    sh_cones_.push_back(
        mrs_lib::SubscribeHandler<rad_msgs::Cone>(shopts, topic_name, &ConeAggregator::callbackTimeout, this, &ConeAggregator::callbackCone, this));
  }

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[ConeAggregator]: initialized");
}

//}

/* callbackCone() //{ */

void ConeAggregator::callbackCone(const rad_msgs::Cone::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  // transform the cone to the

  rad_msgs::Cone cone_tfed_;

  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose   = msg->pose;

  std::string frame_id  = msg->header.frame_id;
  std::size_t slash_pos = frame_id.find("/");

  if (slash_pos != std::string::npos) {
    pose.header.frame_id = _uav_name_ + frame_id.substr(slash_pos, frame_id.size() - 1);
    ROS_INFO("[ConeAggregator]: transforming cone from %s", frame_id.c_str());
  }

  geometry_msgs::Vector3Stamped direction;

  direction.header   = pose.header;
  direction.vector.x = msg->direction.x;
  direction.vector.y = msg->direction.y;
  direction.vector.z = msg->direction.z;

  auto res  = transformer_->transformSingle(pose, _target_frame_);
  auto res2 = transformer_->transformSingle(direction, _target_frame_);

  if (res && res2) {

    cone_tfed_.header = res->header;
    cone_tfed_.pose   = res->pose;

    cone_tfed_.direction.x = res2->vector.x;
    cone_tfed_.direction.y = res2->vector.y;
    cone_tfed_.direction.z = res2->vector.z;

    cone_tfed_.angle = msg->angle;

    /* batch_vizualizer_.clearBuffers(); */
    /* batch_vizualizer_.clearVisuals(); */

    mrs_lib::geometry::Cone cone_vis(Eigen::Vector3d(cone_tfed_.pose.position.x, cone_tfed_.pose.position.y, cone_tfed_.pose.position.z), cone_tfed_.angle,
                                     cos(cone_tfed_.angle) * 10.0, Eigen::Vector3d(cone_tfed_.direction.x, cone_tfed_.direction.y, cone_tfed_.direction.z));

    batch_vizualizer_.addCone(cone_vis, 0.3, 0.8, 0.5, 0.6, true, false, 30, ros::Duration(2.0));
    /* batch_vizualizer_.addCone(cone_vis, 0.0, 0.0, 0.0, 0.3, false, false, 30, ros::Duration(2.0)); */

    batch_vizualizer_.publish();

  } else {

    ROS_ERROR_THROTTLE(1.0, "[ConeAggregator]: could not transform incoming cone from %s to %s", msg->header.frame_id.c_str(), _target_frame_.c_str());
    return;
  }

  ph_cones_.publish(cone_tfed_);
}

//}

/* callbackTimeout() //{ */
void ConeAggregator::callbackTimeout(const std::string &topic_name, const ros::Time &last_msg_time) {
  if (!is_initialized_) {
    return;
  }
  ROS_ERROR("[ConeAggregator]: Did not receive a message on topic %s for %.2f seconds", topic_name.c_str(), (ros::Time::now() - last_msg_time).toSec());
}
//}

}  // namespace compton_cone_generator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_cone_generator::ConeAggregator, nodelet::Nodelet)
