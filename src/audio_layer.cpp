#include "audio_costmap_plugin/audio_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include <cmath>
#include <vector>
#include <array>

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace audio_costmap_plugin
{

  AudioLayer::AudioLayer()
      : last_min_x_(-std::numeric_limits<float>::max()),
        last_min_y_(-std::numeric_limits<float>::max()),
        last_max_x_(std::numeric_limits<float>::max()),
        last_max_y_(std::numeric_limits<float>::max())
  {
  }

  // This method is called at the end of plugin initialization.
  // It contains ROS parameter(s) declaration and initialization
  // of need_recalculation_ variable.
  void AudioLayer::onInitialize()
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);
    // node->get_parameter(name_ + "." + "danger_threshold", danger_threshold);

    observer_sub = node->create_subscription<geometry_msgs::msg::PoseArray>("/observers", rclcpp::SensorDataQoS(), std::bind(&AudioLayer::observerCallback, this, std::placeholders::_1));
    publisher_ = node->create_publisher<sensor_msgs::msg::Image>("audio_map", 10);

    need_recalculation_ = false;
    current_ = true;
  }

  void AudioLayer::observerCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    stealth_message_mutex.lock();
    observerList = *msg;
    stealth_message_mutex.unlock();
  }

  // The method is called to ask the plugin: which area of costmap it needs to update.
  // Inside this method window bounds are re-calculated if need_recalculation_ is true
  // and updated independently on its value.
  void
  AudioLayer::updateBounds(
      double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
      double *min_y, double *max_x, double *max_y)
  {
    if (need_recalculation_)
    {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;

      *min_x = -std::numeric_limits<float>::max();
      *min_y = -std::numeric_limits<float>::max();
      *max_x = std::numeric_limits<float>::max();
      *max_y = std::numeric_limits<float>::max();
      need_recalculation_ = false;
    }
    else
    {
      double tmp_min_x = last_min_x_;
      double tmp_min_y = last_min_y_;
      double tmp_max_x = last_max_x_;
      double tmp_max_y = last_max_y_;
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      *min_x = std::min(tmp_min_x, *min_x);
      *min_y = std::min(tmp_min_y, *min_y);
      *max_x = std::max(tmp_max_x, *max_x);
      *max_y = std::max(tmp_max_y, *max_y);
    }
  }

  // The method is called when footprint was changed.
  // Here it just resets need_recalculation_ variable.
  void
  AudioLayer::onFootprintChanged()
  {
    need_recalculation_ = true;
  }

  double getDistance(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x1 - x2, 2.0) + std::pow(y1 - y2, 2.0));
  }

  double normalize(double value, double min, double max)
  {
    return (value - min) / (max - min);
  }

  inline float inv_lerp(float a, float b, float v)
  {
    return (v - a) / (b - a);
  }

  inline unsigned char audio_lerp(unsigned char a, unsigned char b, float t)
  {
    return static_cast<char>((static_cast<float>((b-a)) * t)) + a;
  }

  unsigned char convert_to_scale(float max_safe_dB, float safe_dB, float cell_dB)
  {
    if (cell_dB <= safe_dB) return 0;
    if (cell_dB >= max_safe_dB) return nav2_costmap_2d::MAX_NON_OBSTACLE;
    // simple LERP to start
    float percentage = inv_lerp(max_safe_dB, safe_dB, cell_dB);
    unsigned char cost = audio_lerp(0u, nav2_costmap_2d::MAX_NON_OBSTACLE, percentage);
    return cost;
  }

  

  // Computes the cost for a given cell utilizing the observers
  // current locations. Used in updateCosts
  //
  //
  unsigned char AudioLayer::getCost(nav2_costmap_2d::Costmap2D &master_grid, int mx, int my)
  {
    size_t index = master_grid.getIndex(mx, my);
    if (master_grid.getCost(index) >= LETHAL_OBSTACLE)
    {
      return master_grid.getCost(index);
    }

    float blurred_cost = audio_map.get_blurred_cost(mx, my);
    unsigned char additional_cost = convert_to_scale(40, 20, blurred_cost);

    return nav2_costmap_2d::MAX_NON_OBSTACLE;
    unsigned char total_cost = master_grid.getCost(index) + additional_cost;
    if (total_cost < master_grid.getCost(index)) return nav2_costmap_2d::MAX_NON_OBSTACLE; // check for wrap around
    return total_cost;
  }

  // The method is called when costmap recalculation is required.
  // It updates the costmap within its window bounds.
  // Inside this method the costmap stealth is generated and is writing directly
  // to the resulting costmap master_grid without any merging with previous layers.
  void AudioLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    if (!enabled_)
    {
      return;
    }

    if (observerList.poses.size() == 0)
    {
      // we shouldn't run the update if there is no observers...
      return;
    }

    update_volume_map(master_grid, observerList);
    publisher_->publish(audio_map.to_image());

    // master_array - is a direct pointer to the resulting master_grid.
    // master_grid - is a resulting costmap combined from all layers.
    // By using this pointer all layers will be overwritten!
    // To work with costmap layer and merge it with other costmap layers,
    // please use costmap_ pointer instead (this is pointer to current
    // costmap layer grid) and then call one of updates methods:
    // - updateWithAddition()
    // - updateWithMax()
    // - updateWithOverwrite()
    // - updateWithTrueOverwrite()
    // In this case using master_array pointer is equal to modifying local costmap_
    // pointer and then calling updateWithTrueOverwrite():
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
    // These variables are used to update the costmap only within this window
    // avoiding the updates of whole area.
    //
    // Fixing window coordinates with map size if necessary.
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    std::vector<std::vector<unsigned char>> results;
    for (int i = 0; i < max_j - min_j; i++){
      results.push_back(std::vector<unsigned char>(max_i - min_i));
    }

    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "AudioLayer::updateCosts(): Max Volume: %f",
                 audio_map.get_max_volume());

    // Where we actually set the costs
    for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {
        int index = master_grid.getIndex(i, j);

        unsigned char oldCost = master_grid.getCost(index);
        results[j - min_j][i - min_i] = oldCost;

        // If the current cell we are checking is unknown, we should leave it that way
        if (oldCost == nav2_costmap_2d::NO_INFORMATION)
          continue;

        // setting the stealth cost
        // unsigned char cost = std::max(getCost(master_grid, i, j), oldCost);
        // unsigned char cost = getCost(master_grid, i, j);
        float percent_cost = audio_map.get_blurred_cost(i, j) / audio_map.get_max_volume();
        unsigned char cost = static_cast<unsigned char>(percent_cost * 255.0f);
        results[j - min_j][i - min_i] = std::max(cost, oldCost);
        // master_grid.setCost(i, j, cost);
      }
    }

    for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {
        master_grid.setCost(i, j, results[j - min_j][i - min_i]);
      }
    }
  }

  void AudioLayer::update_volume_map(nav2_costmap_2d::Costmap2D & master_grid, geometry_msgs::msg::PoseArray& observer_positions)
  {
    audio_map.set_map(master_grid);
    // std::vector<std::array<float, 2>> observer_positions = {{ 0.5, -0.5 }}; // TODO: Use the subscribed version to do this
    audio_map.update_costs(observer_positions, 4, 50);
  }

} // namespace audio_costmap_plugin

// This is the macro allowing a audio_costmap_plugin::AudioLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(audio_costmap_plugin::AudioLayer, nav2_costmap_2d::Layer)