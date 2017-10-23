/***************************************************************************
 * Copyright (c) 2017
 * Florian Beck <florian.beck@tuwien.ac.at>
 * Markus Bader <markus.bader@tuwien.ac.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the TU-Wien.
 * 4. Neither the name of the TU-Wien nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Markus Bader ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************/

#ifndef OBJECT_FILTER_H
#define OBJECT_FILTER_H

#include <ros/ros.h>
#include <filters/filter_base.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <tuw_object_msgs/ObjectDetection.h>

namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan
 *        in a circle around a detected object
 */
class LaserScanObjectFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  LaserScanObjectFilter();
  bool configure();

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);

  void detectionCallback(const tuw_object_msgs::ObjectDetection& detection);

private:
  bool insideObject(tf::Point& point_object, tf::Point& point_laser);

  laser_geometry::LaserProjection projector_;

  // tf listener to transform scans into the box_frame
  tf::TransformListener tf_listener_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_detection_;
  tuw_object_msgs::ObjectDetection detection_;

  double object_radius_;
  std::string detection_topic_name_;
};
}

#endif /* OBJECT_FILTER_H */
