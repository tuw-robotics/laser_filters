/***************************************************************************
 * Copyright (c) 2017
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

#include "laser_filters/line_filter.h"
#include <ros/ros.h>
#include <cfloat>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuw_geometry_msgs/LineSegment.h>
#include <tuw_geometry_msgs/LineSegments.h>

#include <tuw_linedetection/Linesegment2DDetectorConfig.h>
using namespace cv;
using namespace std;


laser_filters::LaserLineFilter::LaserLineFilter() {

}


void laser_filters::LaserLineFilter::callbackConfig ( laser_filters::LineFilterConfig &config, uint32_t level ) {
    config_ = config;
}

bool laser_filters::LaserLineFilter::configure() {

    reconfigureServer_ = std::make_shared< dynamic_reconfigure::Server< laser_filters::LineFilterConfig> > ( ros::NodeHandle ( "~/LineFilter" ) );
    reconfigureFnc_ = boost::bind ( &laser_filters::LaserLineFilter::callbackConfig, this,  _1, _2 );
    reconfigureServer_->setCallback ( reconfigureFnc_ );
    line_pub_ = nh_.advertise<tuw_geometry_msgs::LineSegments> ( "line_segments", 1000 );

    return true;

}

bool laser_filters::LaserLineFilter::update (
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan ) {
    size_t nr =  input_scan.ranges.size();
    double scale = config_.scan_pixel_resolution;
    double width = scale * ( input_scan.range_max * 2.1 );
    double center = width/2;
    img_scan_.create ( width, width, CV_8U );
    img_scan_.setTo ( 0 );
    Point p;
    for ( size_t i = 0; i < nr; i++ ) {
        double length = input_scan.ranges[i];

        if ( ( length < input_scan.range_max ) && isfinite ( length ) ) {
            double angle  = input_scan.angle_min + ( input_scan.angle_increment * i );
            p.x = round ( -cos ( angle ) * length * scale + center );
            p.y = round ( -sin ( angle ) * length * scale + center );
            img_scan_.at<uchar> ( p.x, p.y ) = 255;
        }

    }
    for ( size_t i = 0; i < output_scan.ranges.size(); i++ ) {
        output_scan.ranges[i] = 2.;
    }
    vector<Vec4i> lines;
    HoughLinesP ( img_scan_, lines, config_.rho, config_.theta, config_.threshold, config_.minLineLength*scale, config_.maxLineGap*scale );
    if ( config_.plot_detected_lines ) {
        cvtColor ( img_scan_, img_lines_, CV_GRAY2BGR );
        for ( size_t i = 0; i < lines.size(); i++ ) {
            Vec4i l = lines[i];
            line ( img_lines_, Point ( l[0], l[1] ), Point ( l[2], l[3] ), Scalar ( 0,0,255 ), 3, CV_AA );
        }
        imshow ( "detected lines", img_lines_ );
        waitKey ( 10 );
    }


    tuw_geometry_msgs::LineSegment line_segment_msg;
    tuw_geometry_msgs::LineSegments line_segments_msg;
    for ( int i = 0; i < lines.size(); i++ ) {
        Vec4i l = lines[i];
        line_segment_msg.p0.x = -(l[1] - center) / scale;
        line_segment_msg.p0.y = -(l[0] - center) / scale;
        line_segment_msg.p0.z = 0;
        line_segment_msg.p1.x = -(l[3] - center) / scale;
        line_segment_msg.p1.y = -(l[2] - center) / scale;
        line_segment_msg.p1.z = 0;
        line_segments_msg.segments.push_back ( line_segment_msg );
    }
    // set header information
    line_segments_msg.header = input_scan.header;

    line_pub_.publish ( line_segments_msg );


    return true;
}

