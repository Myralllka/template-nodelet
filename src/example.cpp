#include <CLASS_NAME.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace NAMESPACE_NAME
{

/* onInit() method //{ */
void CLASS_NAME::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here

  /* obtain node handle */
  ros::NodeHandle nh("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(nh, "CLASS_NAME");

  param_loader.loadParam("UAV_NAME", _uav_name_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = mrs_lib::Transformer("CLASS_NAME", _uav_name_);

  pub_example     = nh.advertise<std_msgs::UInt64>("test_publisher", 1);

  // | -------------------- initialize timers ------------------- |

  ROS_INFO_ONCE("[CLASS_NAME]: initialized");

  is_initialized_ = true;
}
//}

// | ---------------------- msg callbacks --------------------- |


// | --------------------- timer callbacks -------------------- |

// | -------------------- other functions ------------------- |

}  // namespace NAMESPACE_NAME  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(NAMESPACE_NAME::CLASSNAME, nodelet::Nodelet);
