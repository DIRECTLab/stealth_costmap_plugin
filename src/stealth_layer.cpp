#include "stealth_costmap_plugin/stealth_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include <cmath>
#include <vector>

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace stealth_costmap_plugin
{

  StealthLayer::StealthLayer()
      : last_min_x_(-std::numeric_limits<float>::max()),
        last_min_y_(-std::numeric_limits<float>::max()),
        last_max_x_(std::numeric_limits<float>::max()),
        last_max_y_(std::numeric_limits<float>::max())
  {
  }

  // This method is called at the end of plugin initialization.
  // It contains ROS parameter(s) declaration and initialization
  // of need_recalculation_ variable.
  void
  StealthLayer::onInitialize()
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    observer_sub = node->create_subscription<geometry_msgs::msg::PoseArray>("/observers", rclcpp::SensorDataQoS(), std::bind(&StealthLayer::observerCallback, this, std::placeholders::_1));

    need_recalculation_ = false;
    current_ = true;
  }

  void StealthLayer::observerCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    stealth_message_mutex.lock();
    observerList = *msg;
    stealth_message_mutex.unlock();
  }

  // The method is called to ask the plugin: which area of costmap it needs to update.
  // Inside this method window bounds are re-calculated if need_recalculation_ is true
  // and updated independently on its value.
  void
  StealthLayer::updateBounds(
      double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
      double *min_y, double *max_x, double *max_y)
  {
    if (need_recalculation_)
    {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      // For some reason when I make these -<double>::max() it does not
      // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
      // -<float>::max() instead.
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
  StealthLayer::onFootprintChanged()
  {
    need_recalculation_ = true;

    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "StealthLayer::onFootprintChanged(): num footprint points: %lu",
                 layered_costmap_->getFootprint().size());
  }

  double getDistance(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x1 - x2, 2.0) + std::pow(y1 - y2, 2.0));
  }

  double normalize(double value, double min, double max)
  {
    return (value - min) / (max - min);
  }



  // The actual meat of the code for the bresenham wall checking. This shouldn't be called directly
  unsigned int _bresenhamHelper(nav2_costmap_2d::Costmap2D &master_grid, int mx1, int my1, int mx2, int my2, bool backwards)
  {
    int dx = std::abs(mx2 - mx1);
    int dy = std::abs(my2 - my1);

    int x = mx1;
    int y = my1;

    int pk = 2 * dy - dx;

    unsigned int wallCount = 0;

    for (int i = 0; i < dx + 1; i++)
    {
      int currentIndex = backwards ? master_grid.getIndex(y, x) : master_grid.getIndex(x, y);
      if (master_grid.getCost(currentIndex) > nav2_costmap_2d::MAX_NON_OBSTACLE)
      {
        wallCount++;
      }

      if (mx1 < mx2) x++;
      else x--;

      if (pk < 0) pk += 2 * dy;
      else
      {
        if (my1 < my2) y++;
        else y--;

        pk += 2 * dy - 2 * dx;
      }
    }

    return wallCount;
  }

  // Uses Bresenham Line Algorithm to compute how many wall cells are between the given positions
  // This is useful for checking line of sight, as well as for audio, the thickness of walls between
  // the 2 positions.
  // This is just the kick off function, the real work happens in the helper
  unsigned int bresenhamWallCheck(nav2_costmap_2d::Costmap2D &master_grid, int mx1, int my1, int mx2, int my2)
  {
    int dx = std::abs(mx2 - mx1);
    int dy = std::abs(my2 - my1);

    if (dx < dy)
      return _bresenhamHelper(master_grid, my1, mx1, my2, mx2, true);
    else
      return _bresenhamHelper(master_grid, mx1, my1, mx2, my2, false);
  }

  // Computes the cost for a given cell utilizing the observers
  // current locations. Used in updateCosts
  //
  //
  unsigned char StealthLayer::getCost(nav2_costmap_2d::Costmap2D &master_grid, int mx, int my)
  {
    // Get the position of the current cell in world coordinates
    double wx, wz;
    master_grid.mapToWorld(mx, my, wx, wz);

    unsigned char maxCost = 0;

    for (auto observer : observerList.poses)
    {
      double distance = getDistance(wx, wz, observer.position.x, observer.position.z);
      double normalizedDistance = normalize(distance, 0, VIEW_DISTANCE_METERS);
      if (normalizedDistance > 1)
      {
        continue;
      }

      // Get number of obstacles between observer and current point
      unsigned int omx, omy; // observer map position
      master_grid.worldToMap(observer.position.x, observer.position.z, omx, omy);
      unsigned int numObstaclesBetween = bresenhamWallCheck(master_grid, mx, my, omx, omy);

      // Invert it, so closer is higher cost
      normalizedDistance = std::abs(normalizedDistance - 1.0);

      if (numObstaclesBetween > 0) normalizedDistance = 0.0;

      if (normalizedDistance > 0){
        normalizedDistance += 0.05;
        normalizedDistance = std::min(normalizedDistance, 1.0);
      }

      unsigned char cost = static_cast<unsigned char>(normalizedDistance * nav2_costmap_2d::LETHAL_OBSTACLE);
      if (cost > maxCost)
      {
        maxCost = cost;
      }
    }
    return maxCost;
  }

  // The method is called when costmap recalculation is required.
  // It updates the costmap within its window bounds.
  // Inside this method the costmap stealth is generated and is writing directly
  // to the resulting costmap master_grid without any merging with previous layers.
  void
  StealthLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
      int max_i,
      int max_j)
  {
    if (!enabled_)
    {
      return;
    }

    // No cost modification if there's no observers
    if (observerList.poses.size() == 0)
    {
      return;
    }

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
        unsigned char cost = std::max(getCost(master_grid, i, j), oldCost);
        results[j - min_j][i - min_i] = cost;
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

} // namespace nav2_stealth_costmap_plugin

// This is the macro allowing a nav2_stealth_costmap_plugin::StealthLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(stealth_costmap_plugin::StealthLayer, nav2_costmap_2d::Layer)