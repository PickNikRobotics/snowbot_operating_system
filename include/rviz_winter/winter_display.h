/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, David Lu!!
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_WINTER_WINTER_DISPLAY_H
#define RVIZ_WINTER_WINTER_DISPLAY_H

#include <rviz/display.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <geometry_msgs/Point.h>
#include <vector>

namespace rviz_winter
{
inline double randScale()
{
  return static_cast<double>(rand()) / RAND_MAX;
}

class WinterDisplay: public rviz::Display
{
Q_OBJECT
public:
  WinterDisplay();

  void update(float wall_dt, float ros_dt) override;

protected:
  void onInitialize() override;

private Q_SLOTS:
  void updateSize();
  void updatePosition();

private:
  void letItSnow();

  void initializeXY(geometry_msgs::Point& pt) const;

  rviz::PointCloud* point_cloud_;
  std::vector<rviz::PointCloud::Point> flakes_;

  std::vector<geometry_msgs::Point> points_;

  rviz::FloatProperty* width_property_;
  rviz::FloatProperty* height_property_;
  rviz::FloatProperty* gravity_property_;
  rviz::FloatProperty* wind_property_;
  rviz::FloatProperty* jiggle_property_;
  rviz::IntProperty* size_property_;

  double width_, height_;
};
}  // namespace rviz_winter

#endif  // RVIZ_WINTER_WINTER_DISPLAY_H
