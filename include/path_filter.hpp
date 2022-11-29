#include <vector>
#include "../include/catmull_rom.hpp"

class PathFilter {

private:
	CatmullRom spline_object;
	std::vector<float> distance_cache;
	float deviation;

	Coordinates request_coordinates(int coordinates_offset, float spline_progress);
	int get_base_coordinate(float chained_distance);

public:
	PathFilter(CatmullRom spline_object, float deviation);
	Coordinates get_next(float chained_distance, float sight_range);
	float get_chained_distance(int coordinates_offset, float spline_progress);
	Coordinates get_chained_coordinates(float chained_distance);
};
