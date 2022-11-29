#include <cmath>
#include <vector>
#include "../include/catmull_rom.hpp"
#include "../include/constants.h"

CatmullRom::CatmullRom() {
	this->coordinates = std::vector<Coordinates>();
}

CatmullRom::CatmullRom(std::vector<Coordinates> coordinates) {
	this->coordinates = coordinates;
}

/**
 * Obtain the coordinate on the spline
 *
 * @param coordinate_offset the offset (index) of coordinate objects
 * @param spline_progress the progress on the spline
 * @returns the coordinate on the spline
 */
Coordinates CatmullRom::get_coordinates(int coordinate_offset, float spline_progress) {
	float x_coordinate = this->get_coordinate(coordinate_offset, Coordinates::CoordinateType::X_COORDINATE, spline_progress);
	float y_coordinate = this->get_coordinate(coordinate_offset, Coordinates::CoordinateType::Y_COORDINATE, spline_progress);
	Coordinates combined_coordinates(x_coordinate, y_coordinate, 0.0f);
	return combined_coordinates;
}

/**
 * Obtain the x/y coordinate of a point on spline
 *
 * @param coordinate_offset the offset (index) of coordinate objects
 * @param coordinate_type the type (x/y) of coordinate to calculate
 * @param spline_progress the progress on the spline
 * @returns the x/y coordinate of a point on spline
 */
float CatmullRom::get_coordinate(int coordinate_offset, Coordinates::CoordinateType coordinate_type, float spline_progress) {
	float influence_sum = 0;
	for (int coordinate_index = 0; coordinate_index < 4; coordinate_index++) {
		Coordinates loop_coordinate = this->coordinates[coordinate_offset + coordinate_index];
		// get the x/y coordinate of coordinate objects (4) according to coordinate_type
		float loop_coordinate_type_specified = coordinate_type == Coordinates::CoordinateType::X_COORDINATE ? loop_coordinate.get_x() : loop_coordinate.get_y();
		// apply the influence to each coordinates
		influence_sum += loop_coordinate_type_specified * this->get_degree_influence(coordinate_index, spline_progress);
	}
	return influence_sum;
}

/**
 * Obtain the influence of certain degree
 *
 * @param influence_degree the degree of influence
 * @param spline_progress the progress on the spline
 * @returns the calculated influence for degree
 */
float CatmullRom::get_degree_influence(int influence_degree, float spline_progress) {
	float influence_sum = 0;
	for (int degree_index = 0; degree_index < 4; degree_index++) {
		influence_sum += Constants::CatmullRom::INFLUENCE_RATIO[influence_degree][degree_index] * std::pow(spline_progress, degree_index);
	}
	return influence_sum;
}

/**
 * Obtain the anchor coordinates
 *
 * @returns the anchor coordinates
 */
std::vector<Coordinates> CatmullRom::get_anchors() {
	return this->coordinates;
}
