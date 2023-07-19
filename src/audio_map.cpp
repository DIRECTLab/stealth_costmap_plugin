#include "audio_costmap_plugin/audio_map.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"
#include <algorithm>
#include <iostream>

vec2i vec2i::subtract(vec2i& left, vec2i& right) {
	return {left.x - right.x, left.y - right.y};
}

vec2i vec2i::add(vec2i& left, vec2i& right) {
	return {left.x + right.x, left.y + right.y};
}

vec2i vec2i::abs(vec2i& val) {
  return {std::abs(val.x), std::abs(val.y)};
}


vec2i vec2i::swap(vec2i& val) {
  return { val.y, val.x };
}

void AudioMap::set_map(nav2_costmap_2d::Costmap2D& costmap)
{
  map = costmap;
  volume_map.clear();
  width = map.getSizeInCellsX();
  height = map.getSizeInCellsY();
  volume_map.resize(width * height);
}

sensor_msgs::msg::Image AudioMap::to_image()
{
  sensor_msgs::msg::Image out_image;
  out_image.encoding = sensor_msgs::image_encodings::RGB8;
  out_image.width = static_cast<uint32_t>(width);
  out_image.height = static_cast<uint32_t>(height);
  out_image.is_bigendian = 0u;
  out_image.step = sizeof(unsigned char) * out_image.width * 3; // RGB
  out_image.header = std_msgs::msg::Header();

  std::vector<unsigned char> data(width * height * 3);
  float max = get_max_volume();
  int index = 0;
  for (size_t y = 0; y < height; y++)
  {
    for (size_t x = 0; x < width; x++)
    {
      // float val = get_blurred_cost(x, y) > 0 ? 1.0 : 0.0;
      float val = get_blurred_cost(x, y) / max;
      unsigned char red = static_cast<unsigned char>(val * 255);
      unsigned char green = 255u - red;
      unsigned char blue = 0;
      if (val == 0)
      {
        red = 0u;
        green = 0u;
      }
      if (is_wall(x, y))
      {
        red = 255u;
        green = 255u;
        blue = 255u;
      }
      data.at(index) = red;
      index++;
      data.at(index) = green;
      index++;
      data.at(index) = blue;
      index++;
    }
  }
  out_image.data = data;

  return out_image;
}

bool AudioMap::is_valid(size_t x, size_t y)
{
  return x < width && y < height;
}

bool AudioMap::is_vertical_wall(size_t x, size_t y)
{
  return is_wall(x, y-1) || is_wall(x, y+1);
}

bool AudioMap::is_horizontal_wall(size_t x, size_t y)
{
  return is_wall(x-1, y) || is_wall(x+1, y);
}

bool AudioMap::is_wall(size_t x, size_t y)
{
  return map.getCost(map.getIndex(x, y)) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

float AudioMap::get_blurred_cost(size_t x_center, size_t y_center)
{
  int valid_counts = 0;
	float summation = 0.0f;
	for (int y = -1; y <= 1; y++)
	{
		for (int x = -1; x <= 1; x++)
		{
			if (is_valid(x_center + x, y_center + y))
			{
				valid_counts++;
				summation += volume_level(x_center + x, y_center + y);
			}
		}
	}
  return summation / valid_counts;
}

const float MAX_OFFSET_RADIANS = 0.349f;
vec2i AudioMap::offset_vector(vec2i input_vector)
{
	float random_num = dis(gen);
	float offset = (random_num) * MAX_OFFSET_RADIANS;

	float new_x = std::cos(offset) * input_vector.x - std::sin(offset) * input_vector.y;
	float new_y = std::sin(offset) * input_vector.x + std::cos(offset) * input_vector.y;

	return {static_cast<long>(new_x), static_cast<long>(new_y)};
}

const uint32_t MAX_BOUNCES = 4;
void AudioMap::update_costs(geometry_msgs::msg::PoseArray& observer_positions, uint32_t resolution, float start_dB)
{
  for (auto& observer : observer_positions.poses)
  {
    for (uint32_t i = 0; i < resolution; i++)
    {
      float current_angle = ((2.0 * PI) / resolution) * i + PI / 10;
			float xDir = std::cos(current_angle);
			float yDir = std::sin(current_angle);

			long xDelta = static_cast<long>(xDir * 100); // essentially fixed point integers
			long yDelta = static_cast<long>(yDir * 100); 

      ray cast_ray{};
      cast_ray.current_dB = start_dB;
      cast_ray.direction = {xDelta, yDelta};
      cast_ray.distance_traveled = 1;
      cast_ray.next_double = 2;
      int mx, my;
      map.worldToMapNoBounds(observer.position.x, observer.position.y, mx, my);
      cast_ray.start.x = mx;
      cast_ray.start.y = my;
      uint32_t j = 0;
      for (j = 0; j < MAX_BOUNCES; j++)
      {
        std::cout << "Direction " << j << ": (" << cast_ray.direction.x << ", " << cast_ray.direction.y << ")" << std::endl;
        if (!bresenham(cast_ray))
        {
          break;
        }
        cast_ray.direction = offset_vector(cast_ray.direction);
      }
    }
  }
}

float AudioMap::get_max_volume()
{
  return *std::max_element(volume_map.begin(), volume_map.end());
}

bool AudioMap::bresenham(ray& cast_ray)
{
  if (std::abs(cast_ray.direction.x) < std::abs(cast_ray.direction.y))
  {
    cast_ray = ray::swap_xy(cast_ray);
    return bresenham_helper(cast_ray, true);
  }
  return bresenham_helper(cast_ray, false);
}

const float VOLUME_THRESHOLD = 1.0f;
bool AudioMap::bresenham_helper(ray& cast_ray, bool swapped)
{
  int dx = std::abs(cast_ray.direction.x);
  int dy = std::abs(cast_ray.direction.y);
  int pk = 2 * dy - dx;

  point prev_pos = cast_ray.start;
  while (true)
  {
    if (cast_ray.current_dB < VOLUME_THRESHOLD)
    {
      return false;
    }
    if (swapped)
    {
      if (!is_valid(cast_ray.start.y, cast_ray.start.x))
      {
        return false;
      }
      if (is_wall(cast_ray.start.y, cast_ray.start.x))
      {
        bool flip_x = prev_pos.y != cast_ray.start.y;
        bool flip_y = prev_pos.x != cast_ray.start.x;
        // check instead using the previous position. If the y's are the same, then we should just flip x
        // if x's are same, flips the y's
        // if both are changed, flip both (for now)

        cast_ray.start = prev_pos;
        cast_ray = ray::swap_xy(cast_ray);
        cast_ray.direction.x *= flip_x ? -1 : 1;
        cast_ray.direction.y *= flip_y ? -1 : 1;
        return true;
      }
      volume_map[cast_ray.start.x * width + cast_ray.start.y] = add_volumes(volume_map[cast_ray.start.x * width + cast_ray.start.y], cast_ray.current_dB);
    }
    else
    {
      if (!is_valid(cast_ray.start.x, cast_ray.start.y))
      {
        return false;
      }
      if (is_wall(cast_ray.start.x, cast_ray.start.y))
      {
        cast_ray.direction.x *= cast_ray.start.x != prev_pos.x ? -1 : 1;
        cast_ray.direction.y *= cast_ray.start.y != prev_pos.y ? -1 : 1;
        cast_ray.start = prev_pos;
        return true;
      }
      volume_map[cast_ray.start.y * width + cast_ray.start.x] = add_volumes(volume_map[cast_ray.start.y * width + cast_ray.start.x], cast_ray.current_dB);
    }

    prev_pos = cast_ray.start;

    if (cast_ray.direction.x > 0) cast_ray.start.x++;
    else if (cast_ray.direction.x < 0) cast_ray.start.x--;

    if (pk < 0) pk += 2 * dy;
    else
    {
      if (cast_ray.direction.y > 0) cast_ray.start.y++;
      else if (cast_ray.direction.y < 0) cast_ray.start.y--;
      pk += 2 * dy - 2 * dx;
    }

    cast_ray.distance_traveled += 1;
    while (cast_ray.distance_traveled >= cast_ray.next_double)
    {
      cast_ray.current_dB -= 6;
      cast_ray.next_double *= 2;
    }
  }
}
