/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <rad_msgs/Cone.h>
#include <rad_msgs/ClusterList.h>

#include <Eigen/Eigen>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/batch_visualizer.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <rad_utils/physics.h>

#include <dynamic_reconfigure/server.h>
#include <compton_cone_generator/compton_cone_generatorConfig.h>

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

namespace compton_cone_generator
{

int _coincidence_matching_method_;

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

  bool   _prior_enabled_ = false;
  double _prior_energy_;
  bool   _prior_edges_enabled_ = false;
  double _prior_photon_edge_;
  double _prior_electron_edge_;

  double _min_pixel_distance_;
  bool   _min_pixel_distance_enabled_;

  // | ----------------------- subscribers ---------------------- |

  ros::Subscriber subscriber_cluster_list_;
  void            callbackClusterList(const rad_msgs::ClusterListConstPtr &msg);

  // | ----------------------- publishers ----------------------- |

  ros::Publisher publisher_cones_;
  ros::Publisher publisher_coincidences_;

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
  SingleEvent(const rad_msgs::Cluster &cluster);
  SingleEvent &operator=(const SingleEvent &other);
  SingleEvent(void){};

  std::shared_ptr<rad_msgs::Cluster> cluster;

  bool              operator<(const SingleEvent &other);
  rad_msgs::Cluster operator()(const SingleEvent &event);
};

// constructor
SingleEvent::SingleEvent(const rad_msgs::Cluster &cluster) {

  this->cluster = std::make_shared<rad_msgs::Cluster>(cluster);
}

SingleEvent &SingleEvent::operator=(const SingleEvent &other) {

  if (this == &other) {
    return *this;
  }

  this->cluster = other.cluster;

  return *this;

};  // namespace compton_cone_generator

bool SingleEvent::operator<(const SingleEvent &other) {
  if (_coincidence_matching_method_ == 0) {
    return this->cluster->toa < other.cluster->toa;
  } else {
    return this->cluster->id < other.cluster->id;
  }
}

rad_msgs::Cluster SingleEvent::operator()(const SingleEvent &event) {
  return *event.cluster;
}

//}

/* inInit() //{ */

void ComptonConeGenerator::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonConeGenerator]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ComptonConeGenerator");

  param_loader.loadParam("coincidence_matching/method", _coincidence_matching_method_);

  param_loader.loadParam("prior/enabled", _prior_enabled_);
  param_loader.loadParam("prior/energy", _prior_energy_);
  param_loader.loadParam("prior/edges/enabled", _prior_edges_enabled_);
  param_loader.loadParam("prior/edges/photon", _prior_photon_edge_);
  param_loader.loadParam("prior/edges/electron", _prior_electron_edge_);

  param_loader.loadParam("min_pixel_distance/enabled", _min_pixel_distance_enabled_);
  param_loader.loadParam("min_pixel_distance/distance", _min_pixel_distance_);

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

  publisher_cones_        = nh_.advertise<rad_msgs::Cone>("cones_out", 1);
  publisher_coincidences_ = nh_.advertise<rad_msgs::ClusterList>("coincidences_out", 1);

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

  ROS_INFO_ONCE("[ComptonConeGenerator]: geting cluster list");

  if (msg->clusters.size() == 0) {
    return;
  }

  std::vector<SingleEvent> events;

  // put the cluster into a list
  for (size_t i = 0; i < msg->clusters.size(); i++) {

    SingleEvent event(msg->clusters[i]);

    events.push_back(event);
  }

  // sort list with clusters
  std::sort(events.begin(), events.end());

  // loop throught the clusters
  size_t it = 0;
  while (it < (events.size() - 1)) {

    std::vector<SingleEvent> coincidences;
    coincidences.push_back(events[it]);

    size_t it2 = it + 1;
    for (; it2 < events.size(); it2++) {

      double time_diff = (events[it2].cluster->toa - events[it].cluster->toa);

      if (_coincidence_matching_method_ == 0) {

        if (time_diff <= _time_constant_) {  // coincidence

          if (events[it].cluster->id != events[it2].cluster->id) {
            ROS_WARN("[SingleEvent]: false positive coincidence, id1: %lu, id2: %lu, toa1: %f, toa2: %f, dt: %.2f ns", events[it].cluster->id,
                     events[it2].cluster->id, events[it].cluster->toa, events[it2].cluster->toa, time_diff);
          }

          coincidences.push_back(events[it2]);

        } else {  // not coincidence

          if (events[it].cluster->id == events[it2].cluster->id) {
            ROS_WARN("[SingleEvent]: false negative coincidence, id1: %lu, id2: %lu, toa1: %f, toa2: %f, dt: %.2f ns", events[it].cluster->id,
                     events[it2].cluster->id, events[it].cluster->toa, events[it2].cluster->toa, time_diff);
          }

          break;
        }

      } else {

        if (events[it].cluster->id == events[it2].cluster->id) {  // coincidence

          ROS_INFO("[SingleEvent]: coincidence, id1: %lu, id2: %lu, toa1: %f, toa2: %f, dt: %.2f ns", events[it].cluster->id, events[it2].cluster->id,
                   events[it].cluster->toa, events[it2].cluster->toa, time_diff);
          coincidences.push_back(events[it2]);

        } else {  // not coincidence
          break;
        }
      }
    }

    if (coincidences.size() == 1) {
      ROS_INFO("[SingleEvent]: no coincidence");

      it++;
      continue;
    } else if (coincidences.size() > 2) {

      std::stringstream ss;

      for (size_t j = 0; j < coincidences.size(); j++) {
        ss << coincidences[j].cluster->id << " ";
      }

      ROS_ERROR("[SingleEvent]: %d clusters in coincidence: %s dt: %.2f ns", int(coincidences.size()), ss.str().c_str(),
                coincidences.back().cluster->toa - coincidences.begin()->cluster->toa);

      it = it2 + 1;
      continue;
    } else {

      it = it2 + 1;
    }

    double time_diff = (coincidences[1].cluster->toa - coincidences[0].cluster->toa);
    ROS_INFO("[SingleEvent]: 2-cluster coincidence, cl stamp: %f, id1: %lu, id2: %lu, toa1: %f, toa2: %f, dt: %.2f ns", msg->header.stamp.toSec(),
             coincidences[0].cluster->id, coincidences[1].cluster->id, coincidences[0].cluster->toa, coincidences[1].cluster->toa, time_diff);

    SingleEvent electron;
    SingleEvent photon;

    if (time_diff > 0) {
      electron = coincidences[1];
      photon   = coincidences[0];
    } else {
      electron = coincidences[0];
      photon   = coincidences[1];
    }

    if (_min_pixel_distance_enabled_ &&
        sqrt(pow(photon.cluster->x - electron.cluster->x, 2) + pow(photon.cluster->y - electron.cluster->y, 2)) < _min_pixel_distance_) {

      ROS_WARN("[SingleEvent]: events are too close");
      continue;
    }

    if (_prior_enabled_ &&
        (fabs((photon.cluster->energy + electron.cluster->energy) - _prior_energy_) > (photon.cluster->energy + electron.cluster->energy) * 0.2)) {
      ROS_WARN("[SingleEvent]: energies don't fit, e1 = %.2f, e2 = %.2f", photon.cluster->energy, electron.cluster->energy);
      continue;
    }

    if (_prior_edges_enabled_ && (photon.cluster->energy < _prior_photon_edge_ || electron.cluster->energy > _prior_electron_edge_)) {
      ROS_INFO("[SingleEvent]: edge energy violated");
      continue;
    }

    ROS_INFO("[SingleEvent]: events fit, photon = %.2f, electron = %.2f", photon.cluster->energy, electron.cluster->energy);

    {  // coincidences will be republished as a new cluster list
      rad_msgs::ClusterList cluster_list_coincidences;
      cluster_list_coincidences.header = msg->header;

      cluster_list_coincidences.clusters.push_back(*coincidences[0].cluster);
      cluster_list_coincidences.clusters.push_back(*coincidences[1].cluster);

      publisher_coincidences_.publish(cluster_list_coincidences);
    }

    double z_distance = (time_diff / _time_constant_) * _sensor_thickness_;

    // calculate the cone direction
    Eigen::Vector3d cone_direction = Eigen::Vector3d(electron.cluster->x * _pixel_pitch_ - photon.cluster->x * _pixel_pitch_,
                                                     electron.cluster->y * _pixel_pitch_ - photon.cluster->y * _pixel_pitch_, z_distance);

    cone_direction.normalize();

    // calculate the scattering angle
    auto theta_estimate = getComptonAngle(electron.cluster->energy, photon.cluster->energy);

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

      cone.header.stamp    = electron.cluster->stamp;
      cone.header.frame_id = _world_frame_;  // TODO fix

      // transform the cone direction to the world frame
      geometry_msgs::Vector3Stamped cone_direction_camera;

      cone_direction_camera.header.stamp    = electron.cluster->stamp;
      cone_direction_camera.header.frame_id = msg->header.frame_id;
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

          ROS_ERROR("[ComptonConeGenerator]: could not transform cone direction (frame: %s, stamp: %f) to the world frame (%s)",
                    cone_direction_camera.header.frame_id.c_str(), cone_direction_camera.header.stamp.toSec(), _world_frame_.c_str());
          continue;
        }
      }

      geometry_msgs::PoseStamped cone_pose_camera;

      cone_pose_camera.header.stamp    = electron.cluster->stamp;
      cone_pose_camera.header.frame_id = msg->header.frame_id;
      cone_pose_camera.pose.position.x = 0;
      cone_pose_camera.pose.position.y = 0;
      cone_pose_camera.pose.position.z = 0;

      Eigen::Vector3d e1                = Eigen::Vector3d(1, 0, 0);
      Eigen::Vector3d axis              = e1.cross(cone_direction);
      double          angle             = mrs_lib::geometry::angleBetween(e1, cone_direction);
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

      mrs_lib::geometry::Cone cone_vis(Eigen::Vector3d(cone.pose.position.x, cone.pose.position.y, cone.pose.position.z), theta, cos(cone.angle) * _plotting_cone_height_,
                             Eigen::Vector3d(cone.direction.x, cone.direction.y, cone.direction.z));

      if (_plotting_clear_cones_) {
        batch_vizualizer_.clearBuffers();
        batch_vizualizer_.clearVisuals();
      }

      batch_vizualizer_.addCone(cone_vis, 0.3, 0.8, 0.5, 0.6, true, false, 30);
      batch_vizualizer_.addCone(cone_vis, 0.0, 0.0, 0.0, 0.3, false, false, 30);

      // mirror the cone
      /* mrs_lib::geometry::Cone cone_vis2(Eigen::Vector3d(cone.pose.position.x, cone.pose.position.y, cone.pose.position.z), theta, 10.0, */
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
