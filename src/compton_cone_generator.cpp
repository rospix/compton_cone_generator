/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <rad_msgs/Cone.h>
#include <rad_msgs/ClusterList.h>

#include <Eigen/Eigen>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/geometry_utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/batch_visualizer.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <radiation_utils/physics.h>

#include <dynamic_reconfigure/server.h>
#include <compton_cone_generator/compton_cone_generatorConfig.h>

//}

namespace compton_cone_generator
{

/* ComptonConeGenerator //{ */

class ComptonConeGenerator : public nodelet::Nodelet {

public:
  virtual void onInit();

  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  mrs_lib::Transformer transformer_;

  mrs_lib::BatchVisualizer batch_vizualizer_;

  // | ------------------------- params ------------------------- |

  double      _plotting_cone_height_;
  bool        _plotting_clear_cones_;
  std::string _uav_name_;
  double      _sensor_thickness_;
  double      _time_constant_;
  double      _pixel_pitch_;
  std::string _compton_camera_frame_;
  std::string _world_frame_;
  bool        _prior_enabled_ = false;
  double      _prior_energy_;

  // | ----------------------- subscribers ---------------------- |

  ros::Subscriber subscriber_cluster_list_;
  void            callbackClusterList(const rad_msgs::ClusterListConstPtr &msg);

  // | ----------------------- publishers ----------------------- |

  ros::Publisher publisher_cones_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer main_timer_;
  int        _main_timer_rate_;
  void       mainTimer(const ros::TimerEvent &event);

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                       drs_mutex_;
  typedef compton_cone_generator::compton_cone_generatorConfig drs_config_t;
  typedef dynamic_reconfigure::Server<drs_config_t>            Drs_t;
  boost::shared_ptr<Drs_t>                                     reconfigure_server_;
  void                                                         callbackDrs(compton_cone_generator::compton_cone_generatorConfig &drs_config_t, uint32_t level);

  compton_cone_generator::compton_cone_generatorConfig drs_params_;
  std::mutex                                           mutex_params_;

  // | ------------------------ routines ------------------------ |

  std::optional<double> getComptonAngle(const double _ee, const double _ef);
};
//}

/* class SingleEvent //{ */

class SingleEvent {

public:
  SingleEvent(double toa, double energy, double x, double y);
  SingleEvent &operator=(const SingleEvent &other);
  SingleEvent(void){};

  double toa;
  double energy;
  double x;
  double y;

  bool operator<(const SingleEvent &other);
};

// constructor
SingleEvent::SingleEvent(double toa, double energy, double x, double y) {

  this->toa    = toa;
  this->energy = energy;
  this->x      = x;
  this->y      = y;
}

SingleEvent &SingleEvent::operator=(const SingleEvent &other) {

  if (this == &other) {
    return *this;
  }

  this->toa    = other.toa;
  this->energy = other.energy;
  this->x      = other.x;
  this->y      = other.y;

  return *this;

};  // namespace compton_cone_generator

bool SingleEvent::operator<(const SingleEvent &other) {
  return this->toa < other.toa;
}

//}

/* inInit() //{ */

void ComptonConeGenerator::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonConeGenerator]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ComptonConeGenerator");

  param_loader.loadParam("prior/enabled", _prior_enabled_);
  param_loader.loadParam("prior/energy", _prior_energy_);

  param_loader.loadParam("plotting/cone_height", _plotting_cone_height_);
  param_loader.loadParam("plotting/clear_cones", _plotting_clear_cones_);

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("world_frame", _world_frame_);
  param_loader.loadParam("compton_camera_frame", _compton_camera_frame_);

  param_loader.loadParam("main_timer_rate", _main_timer_rate_);

  param_loader.loadParam("detector/sensor_thickness", _sensor_thickness_);
  param_loader.loadParam("detector/time_constant", _time_constant_);
  param_loader.loadParam("detector/pixel_pitch", _pixel_pitch_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ComptonConeGenerator]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- transformer ---------------------- |

  transformer_ = mrs_lib::Transformer("ComptonConeGenerator", _uav_name_);

  // | -------------------- batch visualizer -------------------- |

  batch_vizualizer_ = mrs_lib::BatchVisualizer(nh_, "compton_cones", _world_frame_);

  batch_vizualizer_.clearBuffers();
  batch_vizualizer_.clearVisuals();

  // | ----------------------- subscribers ---------------------- |

  subscriber_cluster_list_ = nh_.subscribe("cluster_list_in", 1, &ComptonConeGenerator::callbackClusterList, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_cones_ = nh_.advertise<rad_msgs::Cone>("cones_out", 1);

  // | ------------------------- timers ------------------------- |

  main_timer_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &ComptonConeGenerator::mainTimer, this);

  // | ------------------- dynamic reconfigure ------------------ |

  reconfigure_server_.reset(new Drs_t(drs_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&ComptonConeGenerator::callbackDrs, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[ComptonConeGenerator]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackClusterList() //{ */

void ComptonConeGenerator::callbackClusterList(const rad_msgs::ClusterListConstPtr &msg) {

  if (!is_initialized_)
    return;

  ROS_INFO_STREAM_THROTTLE(1.0, "[ComptonConeGenerator]: geeting cluster list");

  std::vector<SingleEvent> events;

  for (size_t i = 0; i < msg->clusters.size(); i++) {

    SingleEvent event(msg->clusters[i].toa, msg->clusters[i].energy, msg->clusters[i].x, msg->clusters[i].y);

    events.push_back(event);
  }

  std::sort(events.begin(), events.end());

  for (size_t it = 0; it < events.size(); it++) {

    for (size_t it2 = it + 1; it2 < events.size(); it2++) {

      double time_diff = (events[it2].toa - events[it].toa);

      if (fabs(time_diff) > _time_constant_) {
        break;
      }

      SingleEvent electron;
      SingleEvent photon;

      if (time_diff > 0) {
        electron = events[it2];
        photon   = events[it];
      } else {
        electron = events[it];
        photon   = events[it2];
      }

      if (_prior_enabled_ && (fabs((photon.energy + electron.energy) - _prior_energy_) > (photon.energy + electron.energy) * 0.2)) {
        continue;
      } else {
        ROS_INFO("[SingleEvent]: events fit, e1 = %.2f, e2 = %.2f", photon.energy, electron.energy);
      }

      if (photon.energy >= 174.0 && electron.energy <= 477.0) {
        ROS_ERROR("[SingleEvent]: compton is happy");
      } else {
        ROS_ERROR("[SingleEvent]: compton event rejected, energies don't fit");
        continue;
      }

      double z_distance = (time_diff / _time_constant_) * _sensor_thickness_;

      // calculate the cone direction
      Eigen::Vector3d cone_direction =
          Eigen::Vector3d(electron.x * _pixel_pitch_ - photon.x * _pixel_pitch_, electron.y * _pixel_pitch_ - photon.y * _pixel_pitch_, z_distance);

      cone_direction.normalize();

      // calculate the scattering angle
      auto theta_estimate = getComptonAngle(electron.energy, photon.energy);

      if (theta_estimate) {

        double theta = fabs(theta_estimate.value());

        /* cone_direction[0] = 1; */
        /* cone_direction[1] = 0; */
        /* cone_direction[2] = 0; */
        /* cone_direction.normalize(); */

        if (theta > M_PI / 2.0) {
          theta = M_PI - theta;
          cone_direction *= -1.0;
        }

        rad_msgs::Cone cone;

        cone.header.stamp    = ros::Time::now();  // TODO fix
        cone.header.frame_id = _world_frame_;     // TODO fix

        // transform the cone direction to the world frame
        geometry_msgs::Vector3Stamped cone_direction_camera;

        cone_direction_camera.header.stamp    = ros::Time::now();        // TODO fix
        cone_direction_camera.header.frame_id = _compton_camera_frame_;  // TODO fix
        cone_direction_camera.vector.x        = cone_direction[0];
        cone_direction_camera.vector.y        = cone_direction[1];
        cone_direction_camera.vector.z        = cone_direction[2];

        {
          auto result = transformer_.transformSingle(_world_frame_, cone_direction_camera);

          if (result) {

            cone.direction.x = result.value().vector.x;
            cone.direction.y = result.value().vector.y;
            cone.direction.z = result.value().vector.z;

          } else {

            ROS_ERROR("[ComptonConeGenerator]: could not transform cone direction to the world frame");
            continue;
          }
        }

        geometry_msgs::PoseStamped cone_pose_camera;

        cone_pose_camera.header.stamp    = ros::Time::now();        // TODO fix
        cone_pose_camera.header.frame_id = _compton_camera_frame_;  // TODO fix
        cone_pose_camera.pose.position.x = 0;
        cone_pose_camera.pose.position.y = 0;
        cone_pose_camera.pose.position.z = 0;

        Eigen::Vector3d e1                = Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d axis              = e1.cross(cone_direction);
        double          angle             = mrs_lib::vectorAngle(e1, cone_direction);
        cone_pose_camera.pose.orientation = mrs_lib::AttitudeConverter(Eigen::AngleAxis<double>(angle, axis));

        {
          auto result = transformer_.transformSingle(_world_frame_, cone_pose_camera);

          if (result) {

            cone.pose.position.x = result.value().pose.position.x;
            cone.pose.position.y = result.value().pose.position.y;
            cone.pose.position.z = result.value().pose.position.z;

            cone.pose.orientation = result.value().pose.orientation;

          } else {

            ROS_ERROR("[ComptonConeGenerator]: could not transform cone position to the world frame");
            continue;
          }
        }

        cone.angle = theta;

        mrs_lib::Cone cone_vis(Eigen::Vector3d(cone.pose.position.x, cone.pose.position.y, cone.pose.position.z), theta,
                               cos(cone.angle) * _plotting_cone_height_, Eigen::Vector3d(cone.direction.x, cone.direction.y, cone.direction.z));

        if (_plotting_clear_cones_) {
          batch_vizualizer_.clearBuffers();
          batch_vizualizer_.clearVisuals();
        }

        batch_vizualizer_.addCone(cone_vis, 0.2, 0.8, 0.5, 0.8, true, false, 30);
        batch_vizualizer_.addCone(cone_vis, 0.0, 0.0, 0.0, 1.0, false, false, 30);

        // mirror the cone
        /* mrs_lib::Cone cone_vis2(Eigen::Vector3d(cone.pose.position.x, cone.pose.position.y, cone.pose.position.z), theta, 10.0, */
        /*                        Eigen::Vector3d(-cone.direction.x, -cone.direction.y, -cone.direction.z)); */
        /* batch_vizualizer_.addCone(cone_vis2, 0.2, 0.8, 0.5, 0.3, true, false, 30); */

        batch_vizualizer_.publish();

        publisher_cones_.publish(cone);

        ros::Duration(1.0).sleep();

        // mirror the cone
        /* cone.direction.x *= -1; */
        /* cone.direction.y *= -1; */
        /* cone.direction.z *= -1; */
        /* publisher_cones_.publish(cone); */
      }
    }
  }
}

//}

/* callbackDrs() //{ */

void ComptonConeGenerator::callbackDrs(compton_cone_generator::compton_cone_generatorConfig &drs_config_t, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_params_);

    drs_params_ = drs_config_t;
  }

  ROS_INFO("[ComptonConeGenerator]: updated drs params");
}

//}

// | ------------------------- timers ------------------------- |

/* mainTimer() //{ */

void ComptonConeGenerator::mainTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;
}

//}

// | ------------------------ routines ------------------------ |

/* getComptonAngle() //{ */

std::optional<double> ComptonConeGenerator::getComptonAngle(const double _ee, const double _ef) {

  double ee = _ee < 0 ? 0 : _ee;
  double ef = _ef < 0 ? 0 : _ef;

  double e0 = ee + ef;

  /* double eeJ = conversions::energyeVtoJ(ee*1000.0); */
  double efJ = conversions::energyeVtoJ(ef * 1000.0);
  double e0J = conversions::energyeVtoJ(e0 * 1000.0);

  double angle;

  try {
    // Tomas's
    angle = acos(1.0 + constants::me * pow(constants::c, 2.0) * ((1.0 / e0J) - (1.0 / efJ)));

    // Dan's
    /* angle = acos(1.0 - constants::me * pow(constants::c, 2.0) * (eeJ / (e0J * (e0J - eeJ)))); */

    if (!std::isfinite(angle)) {
      ROS_ERROR("NaN detected in variable \"angle\"!!!");
      ROS_ERROR_STREAM("[SingleEvent]: ee = " << ee << " kev");
      ROS_ERROR_STREAM("[SingleEvent]: ef = " << ef << " kev");
      return {};
    }

    return angle;
  }
  catch (...) {
    return {};
  }
}

//}

}  // namespace compton_cone_generator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_cone_generator::ComptonConeGenerator, nodelet::Nodelet)
