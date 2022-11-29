#include <algorithm>
#include <cmath>
#include "../include/coordinates.hpp"

/**
 * Initialize a coordinates object, with given x-coordinate, y-coordinate, and direction
 *
 * @param x_coordinates the x-coordinate of the location
 * @param y_coordinates the y-coordinate of the location
 * @param direction the direction (in degrees) of the location
 */
Coordinates::Coordinates(float x_coordinates, float y_coordinates, float direction) {
	// calculates the direction restricted to boundaries (0~360)
	float restricted_direction = std::fmod(direction, 360.0f);
	if (restricted_direction < 0.0f) restricted_direction += 360.0f;
	// assign coordinates
	float new_coordinates[3] = {x_coordinates, y_coordinates, restricted_direction};
	std::copy(new_coordinates, new_coordinates + 3, this->coordinates);
}

/**
 * Calculates the distance between two coordinates object
 *
 * @param target_coordinates the target coordinates
 * @returns the distance between two coordinates object
 */
float Coordinates::get_distance(Coordinates target_coordinates) {
	float x_difference = target_coordinates.get_x() - this->get_x();
	float y_difference = target_coordinates.get_y() - this->get_y();
	return std::sqrt(std::pow(x_difference, 2) + std::pow(y_difference, 2));
}

/**
 * Calculates the acute angle between two coordinates' directions
 *
 * @param target_coordinates the target coordinates
 * @returns the acute angle between two coordinates' directions
 */
float Coordinates::get_acute(Coordinates target_coordinates) {
	float direction_min = std::min(this->get_direction(), target_coordinates.get_direction());
	float direction_max = std::max(this->get_direction(), target_coordinates.get_direction());
	return std::min(direction_max - direction_min, direction_min - direction_max + 360.0f);
}

/**
 * Returns a new coordinates object with offsets applied
 *
 * @param x_offset the offset in x coordinates
 * @param y_offset the offset in y coordinates
 * @param direction_offset the offset of direction
 * @returns a new coordinates object with offsets applied
 */
Coordinates Coordinates::get_offset(float x_offset, float y_offset, float direction_offset) {
	return Coordinates(this->get_x() + x_offset, this->get_y() + y_offset, this->get_direction() + direction_offset);
}

/**
 * Returns a new coordinates object with x/y coordinates resized by ratio
 *
 * @param resize_ratio the resize ratio
 * @returns a new coordinates object with x/y coordinates resized by ratio
 */
Coordinates Coordinates::get_resize(float resize_ratio) {
	return Coordinates(this->get_x() * resize_ratio, this->get_y() * resize_ratio, this->get_direction());
}

/**
 * Access the x-coordinate of the location
 *
 * @returns the x-coordinate of the location
 */
float Coordinates::get_x() {
	return this->coordinates[0];
}

/**
 * Access the y-coordinate of the location
 *
 * @returns the y-coordinate of the location
 */
float Coordinates::get_y() {
	return this->coordinates[1];
}

/**
 * Access the direction (in degrees) of the location
 *
 * @returns the direction (in degrees) of the location
 */
float Coordinates::get_direction() {
	return this->coordinates[2];
}

/**
 * Calculates the distance between two or more coordinates object
 *
 * @param chained_coordinates the chained coordinates
 * @returns the distance between all the coordinates objects
 */
float Coordinates::get_distance_sum(std::vector<Coordinates> chained_coordinates) {
	float distance_sum = 0;
	for (int coordinate_index = 0; coordinate_index < (chained_coordinates.size() - 1); coordinate_index++) {
		distance_sum += chained_coordinates[coordinate_index].get_distance(chained_coordinates[coordinate_index + 1]);
	}
	return distance_sum;
}


/**
 * Initialize a waypoint object, with given x-coordinate, y-coordinate, direction, linear velocity, and angular velocity
 *
 * @param x_coordinates the x-coordinate of the location
 * @param y_coordinates the y-coordinate of the location
 * @param direction the direction (in degrees) of the location (default is 0 degrees)
 * @param lin_vel the desired linear velocity at the waypoint
 * @param ang_vel the desired linear velocity at the waypoint (default is 0 degrees)
 */
Waypoint::Waypoint(float x_coordinates, float y_coordinates, float direction, float lin_vel, float ang_vel) : Coordinates(x_coordinates, y_coordinates, direction) {
	this->lin_vel = lin_vel;
	this->ang_vel = ang_vel;
}

/**
 * @brief Get desired linear velocity at the waypoint
 *
 * @return desired linear velocity at the waypoint
 */
float Waypoint::get_linear_vel() {
	return this->lin_vel;
}

/**
 * @brief Get desired angular velocity at the waypoint
 *
 * @return desired angular velocity at the waypoint
 */
float Waypoint::get_ang_vel() {
	return this->ang_vel;
}
