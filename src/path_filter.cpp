#include <vector>
#include <cmath>
#include "../include/path_filter.hpp"

/**
 * Initialize a PathFilter object
 *
 * @param spline_object the spline generator object
 * @param deviation the maximum acceptable deviation
 */
PathFilter::PathFilter(CatmullRom spline_object, float deviation) {
	this->spline_object = spline_object;
	this->deviation = deviation;
	this->distance_cache.push_back(0.0f); // 0
	this->distance_cache.push_back(0.0f); // 1
}

// can't skip anchor points
// binary search for finding precision
// use cache
Coordinates PathFilter::get_next(float chained_distance, float sight_range) {
	return this->get_chained_coordinates(chained_distance + sight_range);
}

/**
 * Calculates the chained distance from the beginning of the path
 *
 * @param coordinates_offset the offset of the coordinates
 * @param spline_progress the progress point in the spline
 * @returns the chained distance from the beginning of the path
 */
float PathFilter::get_chained_distance(int coordinates_offset, float spline_progress) {
	if (coordinates_offset <= 1 && spline_progress <= 0) {
		return 0.0f;
	}
	// load distance cache
	int cached_coordinate = std::min((int) this->distance_cache.size() - 1, coordinates_offset);
	float cached_distance = this->distance_cache[cached_coordinate];
	// calculate the remaining distance
	std::vector<Coordinates> chained_coordinates;
	float chained_distance = cached_distance;
	for (int coordinate_index = cached_coordinate; coordinate_index < (coordinates_offset + 1); coordinate_index++) {
		// request points on graph, until reaches progress
		float spline_progress_maximum = coordinate_index < coordinates_offset ? 1.0f : spline_progress;
		for (float progress_index = 0; progress_index <= spline_progress_maximum; progress_index += this->deviation) {
			Coordinates progress_coordinates = this->request_coordinates(coordinate_index, progress_index);
			chained_coordinates.push_back(progress_coordinates);
			// check cache availability
			if (progress_index != 0 || this->distance_cache.size() != coordinate_index) {
				// already cached or somehow couldn't cache
				continue;
			}
			// cache anchor point distance
			chained_distance += Coordinates::get_distance_sum(chained_coordinates);
			this->distance_cache.push_back(chained_distance);
			// clear calculated progress coordinates
			chained_coordinates.clear();
			chained_coordinates.push_back(progress_coordinates);
		}
	}
	// get the distance between every coordinates
	return chained_distance + Coordinates::get_distance_sum(chained_coordinates);
}

/**
 * Obtains the coordinate of a point, with the chained distance from the beginning of the path
 *
 * @param chained_distance the chained distance from the beginning of the path
 * @returns the coordinate of a point with chained distance from the beginning of the path
 */
Coordinates PathFilter::get_chained_coordinates(float chained_distance) {
	int base_coordinate = this->get_base_coordinate(chained_distance);
	float base_distance = this->distance_cache[base_coordinate];
	if (chained_distance == base_distance) {
		// exactly on anchor point
		return this->request_coordinates(base_coordinate, 0.0f);
	}
	// find point by binary search
	float progress_minimum = 0.0f, progress_maximum = 1.0f;
	while (true) {
		float progress_average = (progress_minimum + progress_maximum) / 2.0f;
		float progress_distance = this->get_chained_distance(base_coordinate, progress_average);
		float progress_deviation = std::abs(chained_distance - progress_distance);
		if (progress_deviation <= this->deviation || (progress_maximum - progress_minimum) <= this->deviation) {
			// close enough or no other choice, return it
			return this->request_coordinates(base_coordinate - 1, progress_average);
		} else if (progress_distance < chained_distance) {
			progress_minimum = progress_average;
		}
		else {
			progress_maximum = progress_average;
		}
	}
}

/**
 * Obtain the base coordinate of a point, with the chained distance from the beginning of the path
 *
 * @param chained_distance the chained distance from the beginning of the path
 * @returns the base coordinate of a point, with the chained distance from the beginning of the path
 */
int PathFilter::get_base_coordinate(float chained_distance) {
	int cached_last_coordinate = this->distance_cache.size() - 1;
	float cached_last_distance = this->distance_cache[cached_last_coordinate];
	int maximum_coordinate = (this->spline_object.get_anchors().size() - 2);
	if (cached_last_distance < chained_distance) {
		// request chained distance out of bounds
		if (maximum_coordinate <= cached_last_coordinate) {
			// no more cache available
			return 0;
		}
		// generate cache
		this->get_chained_distance(maximum_coordinate, 0.0f);
	}
	// find maximum base coordinate
	for (int coordinate_index = 2; coordinate_index <= maximum_coordinate; coordinate_index++) {
		if (this->distance_cache[coordinate_index] < chained_distance) {
			continue;
		}
		return coordinate_index - 1;
	}
	return 0;
}

/**
 * Request the coordinate of a point on the spline
 *
 * @param coordinates_offset the offset of the coordinates
 * @param spline_progress the progress point in the spline
 * @returns the coordinate of a point with chained distance from the beginning of the path
 */
Coordinates PathFilter::request_coordinates(int coordinates_offset, float spline_progress) {
	return this->spline_object.get_coordinates(coordinates_offset, spline_progress);
}
