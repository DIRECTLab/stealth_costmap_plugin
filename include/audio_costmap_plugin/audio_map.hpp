#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <iostream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"

// https://www.engineeringtoolbox.com/inverse-square-law-d_890.html
// Every time the distance from a source doubles, the sound drops by 6dB
// This will be taken into account for the initial calculations
// initially, we will assume all the noise is reflected
// and allow 3(?) bounces

struct vec2i {
	long x;
	long y;

	static vec2i subtract(vec2i& left, vec2i& right);
	static vec2i add(vec2i& left, vec2i& right);
	static vec2i abs(vec2i& val);
	static vec2i swap(vec2i& val);
};

using point = vec2i;

struct ray {
	point start;
	vec2i direction;
	float current_dB;
	float distance_traveled;
	float next_double;

	static ray swap_xy(ray& in_ray)
	{
		ray out_ray{};
		out_ray.current_dB = in_ray.current_dB;
		out_ray.direction = vec2i::swap(in_ray.direction);
		out_ray.distance_traveled = in_ray.distance_traveled;
		out_ray.next_double = in_ray.next_double;
		out_ray.start = point::swap(in_ray.start);
		return out_ray;
	}
	
	ray clone()
	{
		ray out_ray{};
		out_ray.current_dB = current_dB;
		out_ray.direction = direction;
		out_ray.distance_traveled = distance_traveled;
		out_ray.next_double = next_double;
		out_ray.start = start;
		return out_ray;
	}

	void print_dir()
	{
		std::cout << "(" << direction.x << ", " << direction.y << ")" << std::endl;
	}
};

const float PI = 3.141592653589793238462643383279502884197169399375;

class AudioMap {
public:
	AudioMap()
	{
		gen = std::mt19937(rd());
		dis = std::uniform_real_distribution<float>(-1, 1);
	}

	float& volume_level(size_t x, size_t y)
	{
		return volume_map[y * width + x];
	}

	static float add_volumes(const float& dB_1, const float& dB_2) // using dBA calculation from https://blog.exair.com/2020/09/03/how-to-add-sound-levels-to-calculate-total-decibels-of-noise/
	{
		return 10.0f * std::log10(std::pow(10, dB_1 / 10) + std::pow(10, dB_2 / 10));
	}

	void update_costs(geometry_msgs::msg::PoseArray& observer_positions, uint32_t resolution, float start_dB);

	bool is_wall(size_t x, size_t y);
	bool is_valid(size_t x, size_t y);

	sensor_msgs::msg::Image to_image();

  void set_map(nav2_costmap_2d::Costmap2D& costmap);
	float get_blurred_cost(size_t x, size_t y);
	float get_max_volume();

private:
	bool bresenham(ray& cast_ray);
	bool bresenham_helper(ray& cast_ray, bool swapped);
	bool is_vertical_wall(size_t x, size_t y);
	bool is_horizontal_wall(size_t x, size_t y);
	vec2i offset_vector(vec2i input_vector);

public:
	std::vector<float> volume_map;
	size_t width = 0;
	size_t height = 0;
private:
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_real_distribution<float> dis;
  nav2_costmap_2d::Costmap2D map;
};