#include "vector.h"

struct Point
{
	Point(float mass_, Vector pos_, float radius_)
	:mass(mass_), pos(pos_), radius(radius_), previousPos(pos_)
	{}
	float mass = 0.f;
	Vector pos = vec_v(0,0);
	Vector vel = vec_v(0,0);
	Vector force = vec_v(0,0);
	float radius = 1;
	Vector previousPos = vec_v(0, 0);
};

struct Spring
{
	Spring(int aID_, int bID_, float stiffnes_, float targetLength_, float damping_)
	:aID(aID_), bID(bID_), stiffnes(stiffnes_), targetLength(targetLength_), damping(damping_)
	{}
	int aID;
	int bID;
	float stiffnes;
	float targetLength;
	float damping;
};

enum ShapeType
{
	SHAPE_CIRCLE,
	SHAPE_POLYGON
};

struct Shape
{
	Shape(std::vector<Vector> vertices_)
	:vertices(vertices_), shapeType(SHAPE_POLYGON){}
	Shape(Vector circlePos_, float circleRadius_)
	:circlePos(circlePos_), circleRadius(circleRadius_), shapeType(SHAPE_CIRCLE){}
	Vector circlePos = vec_v(0, 0);
	float circleRadius = 0;
	std::vector<Vector> vertices;
	int shapeType;
	Vector boundingBoxTL;
	Vector boundingBoxBR;
	Vector polygonMiddlePos;
};

class Softbody
{
public:
	Softbody(Vector pos, float stiffnes, float targetLength, float damping, float pointsMass);
	void add_spring_forces();
	void reset_points_forces();
	void update_body(float dt, float gravity);
	std::vector<Point> points;
	std::vector<Spring> springs;
};

class SoftbodyManager
{
public:
	SoftbodyManager(Vector worldSize_, float gravity_);
	void update_bodies(float dt);
	void add_body(Softbody body);
	Vector world_to_screen_pos(Vector worldPos);
	Vector screen_to_world_pos(Vector screenPos);
	void add_shape(Shape shape);
	bool is_in_bounding_box(Vector v, Vector tl, Vector br);
	bool get_intersections_nb(Vector pos, Shape shape, Vector pointDir);
	Vector closest_point_shape_border(Vector pos, Shape shape);
	Vector closest_point(Vector a, Vector b, Vector p);

	Vector worldSize = vec_v(1,1);
	float gravity = 0.f;
	std::vector<Softbody> bodies;
	std::vector<Shape> shapes;
	Vector debugPoint;
};