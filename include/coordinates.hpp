#include <vector>

class Coordinates {

private:
	float coordinates[3];

public:
	Coordinates(float x_coordinates, float y_coordinates, float direction);
	float get_distance(Coordinates target_coordinates);
	float get_acute(Coordinates target_coordinates);
	float get_x();
	float get_y();
	float get_direction();
	Coordinates get_offset(float x_offset, float y_offset, float direction_offset);
	Coordinates get_resize(float resize_ratio);

	static float get_distance_sum(std::vector<Coordinates> chained_coordinates);

	enum CoordinateType {
		X_COORDINATE,
		Y_COORDINATE,
		DIRECTION
	};
};

class Waypoint : public Coordinates {

private:
	float lin_vel;
	float ang_vel;

public:
	Waypoint(float x_coordinates, float y_coordinates, float lin_vel, float direction = 0, float ang_vel = 0);
	float get_linear_vel();
	float get_ang_vel();
	float get_x();
	float get_y();
	float get_direction();
	float get_distance(Coordinates target_coordinates);
	static float get_distance_sum(std::vector<Coordinates> chained_coordinates);
};
