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

#include "laser_filters/object_filter.h"
#include <tuw_geometry/tuw_geometry.h>
#include <iostream>

using namespace tuw;

namespace laser_filters
{
LaserScanObjectFilter::LaserScanObjectFilter()
{
}

void LaserScanObjectFilter::detectionCallback(const tuw_object_msgs::ObjectDetection &detection)
{
  detection_ = detection;
}

bool LaserScanObjectFilter::configure()
{
  bool detection_topic_name_set = getParam("detection_topic_name", detection_topic_name_);
  
  sub_detection_ = nh_.subscribe(detection_topic_name_, 1, &LaserScanObjectFilter::detectionCallback, this);
  
  bool object_radius_set = getParam("object_radius", object_radius_);

  if (!object_radius_set)
  {
    ROS_ERROR("object_radius_set is not set!");
  }
  
  if (!detection_topic_name_set)
  {
    ROS_ERROR("detection_topic_name is not set!");
  }

  return object_radius_set && detection_topic_name_set;
}

bool LaserScanObjectFilter::update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &output_scan)
{
  output_scan = input_scan;
	
  if(detection_.objects.size() == 0)
  {
    return true;
  }
	
  MeasurementLaser measurement_laser;

  int nr = (input_scan.angle_max - input_scan.angle_min) / input_scan.angle_increment;
  measurement_laser.range_max() = input_scan.range_max;
  measurement_laser.range_min() = input_scan.range_min;
  measurement_laser.resize(nr);
  measurement_laser.stamp() = input_scan.header.stamp.toBoost();
  for (int i = 0; i < nr; i++)
  {
    MeasurementLaser::Beam &beam = measurement_laser[i];
    beam.length = input_scan.ranges[i];
    beam.angle = input_scan.angle_min + (input_scan.angle_increment * i);
    beam.end_point.x() = cos(beam.angle) * beam.length;
    beam.end_point.y() = sin(beam.angle) * beam.length;
  }

  tf::StampedTransform detection2laser;

  try
  {
    tf_listener_.lookupTransform(input_scan.header.frame_id, detection_.header.frame_id, ros::Time(0), detection2laser);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("detectionCallback: %s", ex.what());
    return false;
  }

  for (size_t i = 0; i < detection_.objects.size(); i++)
  {
    tf::Transform p_as_tf;
    tf::poseMsgToTF(detection_.objects[i].object.pose, p_as_tf);
    p_as_tf = detection2laser * p_as_tf;
    tf::Point p_obj = p_as_tf.getOrigin();
    p_obj.setZ(0);

    for (int j = 0; j < nr; j++)
    {
      tf::Point p_laser;
      p_laser.setX(measurement_laser[j].end_point.x());
      p_laser.setY(measurement_laser[j].end_point.y());
      p_laser.setZ(0);

      if (insideObject(p_obj, p_laser))
      {
        output_scan.ranges[j] = input_scan.range_max;
      }
    }
  }

  return true;
}

bool LaserScanObjectFilter::insideObject(tf::Point &point_object, tf::Point &point_laser)
{
  return point_object.distance(point_laser) < object_radius_;
}
}
