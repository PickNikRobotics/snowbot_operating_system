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

#ifndef SNOWBOT_OPERATING_SYSTEM_WINTER_DISPLAY_H
#define SNOWBOT_OPERATING_SYSTEM_WINTER_DISPLAY_H

#include <rviz_common/display.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace snowbot_operating_system
{
inline double randScale()
{
  return static_cast<double>(rand()) / RAND_MAX;
}

class SnowDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  SnowDisplay();

  void update(float wall_dt, float ros_dt) override;

protected:
  void onInitialize() override;

private Q_SLOTS:
  void updateSize();
  void updatePosition();

private:
  void letItSnow();

  void initializeXY(geometry_msgs::msg::Point& pt) const;

  rviz_rendering::PointCloud* point_cloud_;
  std::vector<rviz_rendering::PointCloud::Point> flakes_;

  std::vector<geometry_msgs::msg::Point> points_;

  rviz_common::properties::FloatProperty* width_property_;
  rviz_common::properties::FloatProperty* height_property_;
  rviz_common::properties::FloatProperty* gravity_property_;
  rviz_common::properties::FloatProperty* wind_property_;
  rviz_common::properties::FloatProperty* jiggle_property_;
  rviz_common::properties::IntProperty* size_property_;

  double width_, height_;
};
}  // namespace snowbot_operating_system

#endif  // SNOWBOT_OPERATING_SYSTEM_WINTER_DISPLAY_H
