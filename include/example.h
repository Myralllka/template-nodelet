#pragma once
#ifndef VISION_EXAMPLE_EDGE_DETECT_H
#define VISION_EXAMPLE_EDGE_DETECT_H

/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* TF2 related ROS includes */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

/* camera image messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* some STL includes */
#include <stdlib.h>
#include <cstdio>
#include <mutex>

/* ROS includes for working with OpenCV and images */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

//}

namespace NAMESPACE_NAME
{

/* class CLASS_NAME //{ */
class CLASS_NAME : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */

  /* ros parameters */

  std::string _uav_name_;

  // | --------------------- MRS transformer -------------------- |

  mrs_lib::Transformer transformer_;

  // | ---------------------- msg callbacks --------------------- |

  // | --------------------- timer callbacks -------------------- |

  // | --------- variables, related to message checking --------- |


  // | ----------------------- publishers ----------------------- |

  ros::Publisher             pub_example;

  // | --------------------- other functions -------------------- |

};
//}

}  // namespace NAMESPACE_NAME
#endif
