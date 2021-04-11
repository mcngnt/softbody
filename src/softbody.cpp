#include "softbody.h"

Softbody::Softbody(Vector pos, float stiffnes, float targetLength, float damping, float pointsMass)
{
	points.clear();
	springs.clear();
	Vector rectSize = vec_v(6.67, 3.33);
	Vector pointsSize = vec_v(10, 5);
	for (int x = 0; x < pointsSize.x; ++x)
	{
		for (int y = 0; y < pointsSize.y; ++y)
		{
			points.push_back(Point(pointsMass, vec_v(x/1.5, y/1.5) + pos - rectSize/2, 0.2));
		}
	}
	std::cout << points.size() << std::endl;

	for (int x = 0; x < pointsSize.x; ++x)
	{
		for (int y = 0; y < pointsSize.y; ++y)
		{
			if (y == pointsSize.y - 1)
			{
				continue;
			}
			springs.push_back(Spring(x * pointsSize.y + y, x * pointsSize.y + y + 1, stiffnes, targetLength, damping));
		}
	}

	for (int y = 0; y < pointsSize.y; ++y)
	{
		for (int x = 0; x < pointsSize.x; ++x)
		{
			if (x == pointsSize.x - 1)
			{
				continue;
			}
			springs.push_back(Spring(x * pointsSize.y + y, x * pointsSize.y + y + pointsSize.y, stiffnes, targetLength, damping));
		}
	}

	for (int y = 0; y < pointsSize.y; ++y)
	{
		for (int x = 0; x < pointsSize.x; ++x)
		{
			if (x == pointsSize.x - 1 or y == pointsSize.y - 1)
			{
				continue;
			}
			springs.push_back(Spring(x * pointsSize.y + y, x * pointsSize.y + y + pointsSize.y + 1, stiffnes, targetLength, damping));
		}
	}

	for (int y = 0; y < pointsSize.y; ++y)
	{
		for (int x = 0; x < pointsSize.x; ++x)
		{
			if (x == 0 or y == pointsSize.y - 1)
			{
				continue;
			}
			springs.push_back(Spring(x * pointsSize.y + y, x * pointsSize.y + y - pointsSize.y + 1, stiffnes, targetLength, damping));
		}
	}

}
void Softbody::add_spring_forces()
{
	for (unsigned i = 0; i < springs.size(); ++i)
	{
		Spring mySpring = springs[i];
		Point* a = &points[mySpring.aID];
		Point* b = &points[mySpring.bID];
		float springForce = (dist_v(a->pos, b->pos) - mySpring.targetLength) * mySpring.stiffnes + dot_v(normalize_v(b->pos - a->pos), b->vel - a->vel) * mySpring.damping;
		a->force += normalize_v(b->pos - a->pos) * springForce;
		b->force += normalize_v(a->pos - b->pos) * springForce;
	}
}

void Softbody::reset_points_forces()
{
	for (unsigned i = 0; i < points.size(); ++i)
	{
		points[i].force = vec_v(0, 0);
	}
}

void Softbody::update_body(float dt, float gravity)
{
	reset_points_forces();
	add_spring_forces();
	for (unsigned i = 0; i < points.size(); ++i)
	{
		Point* myPoint = &points[i];
		myPoint->force += vec_v(0, -gravity) * myPoint->mass;
		myPoint->vel += (myPoint->force * dt)/myPoint->mass;
		myPoint->previousPos = myPoint->pos;
		myPoint->pos += myPoint->vel * dt;
	}
}


SoftbodyManager::SoftbodyManager(Vector worldSize_, float gravity_)
:worldSize(worldSize_), gravity(gravity_)
{
	bodies.clear();
	shapes.clear();
}


void SoftbodyManager::update_bodies(float dt)
{
	for (unsigned i = 0; i < bodies.size(); ++i)
	{
		bodies[i].update_body(dt, gravity);

		for (unsigned j = 0; j < bodies[i].points.size(); ++j)
		{
			Point* myPoint = &bodies[i].points[j];
			if (myPoint->pos.y < -worldSize.y/2.f)
			{
				myPoint->vel.y += abs(myPoint->pos.y + worldSize.y/2.f);
				myPoint->vel *= 0.99;
				myPoint->pos.y = -worldSize.y/2.f;
			}

			for (unsigned k = 0; k < shapes.size(); ++k)
			{
				Shape myShape = shapes[k];
				if (myShape.shapeType == SHAPE_CIRCLE)
				{
					float distCircle = dist_v(myPoint->pos, myShape.circlePos);
					if (distCircle < myShape.circleRadius)
					{
						myPoint->vel += normalize_v(myPoint->pos - myShape.circlePos) * (myShape.circleRadius - distCircle);
						myPoint->vel *= 0.99;
						myPoint->pos = normalize_v(myPoint->pos - myShape.circlePos) * myShape.circleRadius + myShape.circlePos;
					}
				}

				if (myShape.shapeType == SHAPE_POLYGON)
				{
					// if (j != 0)
					// {
					// 	continue;
					// }
					if (is_in_bounding_box(myPoint->pos, myShape.boundingBoxTL, myShape.boundingBoxBR))
					{
						if (get_intersections_nb(myPoint->pos, myShape, vec_v(-1, 0))%2 == 1 && get_intersections_nb(myPoint->pos, myShape, vec_v(1, 0))%2 == 1)
						{
							Vector closestPoint = closest_point_shape_border(myPoint->pos, myShape);
							myPoint->vel += closestPoint - myPoint->pos;
							myPoint->pos = closestPoint;
							debugPoint = closestPoint;
						}
					}
				}
			}
		}

		for (unsigned j = 0; j <  bodies[i].springs.size(); ++j)
		{
			Point* a = &bodies[i].points[bodies[i].springs[j].aID];
			Point* b = &bodies[i].points[bodies[i].springs[j].bID];

			float distAB = dist_v(a->pos, b->pos);
			float radiusAB = a->radius + b->radius;
			if (distAB < radiusAB)
			{
				a->vel += normalize_v(a->pos - b->pos) * (radiusAB - distAB);
				b->vel += normalize_v(b->pos - a->pos) * (radiusAB - distAB);
				Vector temp = a->pos;
				a->pos += normalize_v(a->pos - b->pos) * (radiusAB - distAB);
				b->pos += normalize_v(b->pos - temp) * (radiusAB - distAB);
			}
		}
	}

}

Vector SoftbodyManager::world_to_screen_pos(Vector worldPos)
{
	worldPos.y = -worldPos.y;
	return (worldSize/2.f + worldPos)/worldSize * vec_v(SCREEN_W, SCREEN_H);
}

void SoftbodyManager::add_body(Softbody body)
{
	bodies.push_back(body);
}

void SoftbodyManager::add_shape(Shape shape)
{
	Vector middlePos = vec_v(0, 0);
	if (shape.shapeType == SHAPE_POLYGON)
	{
		float minX = std::numeric_limits<float>::max();
		float maxX = std::numeric_limits<float>::min();
		float minY = std::numeric_limits<float>::max();
		float maxY = std::numeric_limits<float>::min();

		for (unsigned i = 0; i < shape.vertices.size(); ++i)
		{
			Vector vertex = shape.vertices[i];
			middlePos += vertex;
			if (vertex.x < minX)
			{
				minX = vertex.x;
			}
			if (vertex.x > maxX)
			{
				maxX = vertex.x;
			}
			if (vertex.y < minY)
			{
				minY = vertex.y;
			}
			if (vertex.y > maxY)
			{
				maxY = vertex.y;
			}
		}
		shape.boundingBoxTL = vec_v(minX, maxY);
		shape.boundingBoxBR = vec_v(maxX, minY);
		shape.polygonMiddlePos = middlePos/(float)shape.vertices.size();
	}
	shapes.push_back(shape);
}

Vector SoftbodyManager::screen_to_world_pos(Vector screenPos)
{
	screenPos.y = SCREEN_H - screenPos.y;
	return (screenPos/vec_v(SCREEN_W, SCREEN_H)) * worldSize - worldSize/2.f;
}

bool SoftbodyManager::is_in_bounding_box(Vector v, Vector tl, Vector br)
{
	return (v.x < br.x) && (v.x > tl.x) && (v.y < tl.y) && (v.y > br.y);
}

bool SoftbodyManager::get_intersections_nb(Vector pos, Shape shape, Vector pointDir)
{
	int intersectionsNB = 0;
	for (unsigned i = 0; i < shape.vertices.size(); ++i)
	{
		Vector shapeLineOrigin = shape.vertices[i];
		Vector shapeLineEnd = vec_v(0, 0);
		if (i == shape.vertices.size()-1)
		{
			shapeLineEnd = shape.vertices[0];	
		}
		else
		{
			shapeLineEnd = shape.vertices[i+1];
		}
		float lineLength = dist_v(shapeLineOrigin, shapeLineEnd);

		Vector shapeLineDir = normalize_v(shapeLineEnd - shapeLineOrigin);
		float crossDir = det_v(pointDir, shapeLineDir);
		if (crossDir == 0)
		{
			continue;
		}
		float t = det_v(shapeLineOrigin - pos, shapeLineDir)/crossDir;
		if (t < 0)
		{
			continue;
		}
		Vector interPoint = pos + pointDir * t;
		if (dist_v(interPoint, shapeLineOrigin) > lineLength or dist_v(interPoint, shapeLineEnd) > lineLength)
		{
			continue;
		}
		intersectionsNB++;
	}
	return intersectionsNB;
}

Vector SoftbodyManager::closest_point_shape_border(Vector pos, Shape shape)
{
	Vector closestPoint = vec_v(0, 0);
	float closestDist = std::numeric_limits<float>::max();
	for (unsigned i = 0; i < shape.vertices.size(); ++i)
	{
		Vector shapeLineOrigin = shape.vertices[i];
		Vector shapeLineEnd = vec_v(0, 0);
		if (i == shape.vertices.size()-1)
		{
			shapeLineEnd = shape.vertices[0];
		}
		else
		{
			shapeLineEnd = shape.vertices[i+1];
		}
		Vector interPoint = closest_point(shapeLineEnd, shapeLineOrigin, pos);
		// Vector interPoint2 = closest_point(shapeLineOrigin, shapeLineEnd, pos);
		// interPoint = (interPoint + interPoint2)/2.f;
		float distInter = dist_v(pos, interPoint);
		if (distInter < closestDist)
		{
			closestDist = distInter;
			closestPoint = interPoint;
		}
	}
	return closestPoint;
}

Vector SoftbodyManager::closest_point(Vector a, Vector b, Vector p)
{
	return a + (b - a) * dot_v(p - a, b - a)/dot_v(b - a, b - a);
}