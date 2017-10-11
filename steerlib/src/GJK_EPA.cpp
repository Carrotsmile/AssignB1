#include "obstacles/GJK_EPA.h"

static const float APPROX = 0.1f;

//personally i'm scared to touch this constructor or use any static variables of any kind.
SteerLib::GJK_EPA::GJK_EPA()
{
}

//finds s_{pt}(direction)
static Util::Vector simplex(Util::Vector& direction, const std::vector<Util::Vector>& _shape) {
	Util::Vector output(0,0,0);
	float max = FLT_MIN;
	for (std::vector<Util::Vector>::const_iterator it = _shape.begin(); it != _shape.end(); it++) {
		float dotp = Util::dot(*it, direction);
		if (max < dotp) {
			max = dotp;
			output = *it;
		}
	}
	return output;
}
//finds s_{pt1 kdiff pt2}(direction) as s_{pt1}(direction) - s_{-pt2}(-direction)
static Util::Vector simplexMinkowski(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector& direction) {
	Util::Vector p1 = simplex(direction, _shapeA);
	Util::Vector p2 = simplex((-1) * direction, _shapeB);
	return p1 - p2;
}

//if simplex is not large enough (less than 3), this changes the direction vector for support vector points
//if simplex is size 3, farthest point is removed from the simplex
//function retur
static bool containsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction) {
	Util::Vector a, b, c;
	if (simplex.size() < 3) {
		a = simplex.back();
		b = simplex.front();

		//slide 55, next direction vector will be norm to the vector between a and b
		c = b - a;
		direction = Util::Vector((-1) * c.z, 0, c.x);

		//make sure it has at least a 90 degree angle with a
		if (Util::dot(c, a) >= 0) {
			direction = -1 * direction;
		}
	}
	else {
		//TODO refactor this to use a function call and save space
		a = simplex.back();
		b = simplex.at(1);
		c = simplex.front();

		//vector of b to a
		Util::Vector b_minus_a = b - a;

		//set direction to norm of b - a
		direction = Util::Vector(-1 * b_minus_a.z, 0, b_minus_a.x);
		direction = Util::dot(direction, c) > 0 ? (-1) * direction : direction;
		if (Util::dot(direction, a) <= 0) {
			//this implies that d's angle with a and c are both more than 90, which means the origin is outside of both lines
			simplex.erase(simplex.begin());
			return false;
		}
		//vector of c to a
		Util::Vector c_minus_a = c - a;
		direction = Util::Vector(-1 * c_minus_a.z, 0, c_minus_a.x);
		direction = Util::dot(direction, b) > 0 ? (-1) * direction : direction;
		if (Util::dot(direction, a) <= 0) {
			simplex.erase(simplex.begin() + 1);
			return false;
		}
		return true;
	}
}

static void getClosestEdge(std::vector<Util::Vector>& simplex, float& minDist,
	std::vector<Util::Vector>::iterator& it, Util::Vector& norm) {
	//TODO refactor for clarity and test
	minDist = FLT_MAX;
	std::vector<Util::Vector>::iterator fit;
	//iterates through each point of the simplex and finds distance from origin to each edge, keeping track of cloest one so far
	for (std::vector<Util::Vector>::iterator git = simplex.begin(); git != simplex.end(); git++) {
		//if last point, go full circle
		if (git + 1 == simplex.end()) {
			fit = simplex.begin();
		}
		else {
			fit = git + 1;
		}

		Util::Vector b_minus_a = *fit - *git;
		Util::Vector a = *git;
		Util::Vector n = Util::normalize(a * (Util::dot(b_minus_a, b_minus_a)) - b_minus_a * (Util::dot(b_minus_a, a)));
		if (Util::dot(n, a) < minDist) {
			minDist = Util::dot(n, a);
			it = fit;
			norm = n;
		}
	}
}

//implementation of GJK algorithm, return true and simplex is not null or false and simplex is nulls
static bool GJK(std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
	Util::Vector direction(-1, 0, 1);
	Util::Vector s = simplexMinkowski(_shapeA, _shapeB, -direction);
	simplex.push_back(s);
	for (;;) {
		Util::Vector w = simplexMinkowski(_shapeA, _shapeB, direction);
		simplex.push_back(w);
		float dotp = Util::dot(w, direction);

		//TODO: test this, should call containsOrigin whenever dotp is positive but i dunno if for sure
		if (dotp >= 0 && containsOrigin(simplex, direction)) {
			return true;
		}
		else if (dotp < 0) {
			return false;
		}
		else {
			continue;
		}
	}
}

//implementation of EPA algorithm, it is given that GJK has concluded that the shapes are intersecting
static void EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, std::vector<Util::Vector>& simplex,
	const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
	float m;
	std::vector<Util::Vector>::iterator it;
	Util::Vector n;

	//should be guranteed to converge
	for (;;) {
		getClosestEdge(simplex, m, it, n);
		Util::Vector p = simplexMinkowski(_shapeA, _shapeB, n);
		float dotp = Util::dot(p, n);
		if (dotp - m <= 0) {
			return_penetration_vector = n;
			return_penetration_depth = dotp;
			return;
		}
		else {
			simplex.insert(it, p);
		}
	}

}
//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simp = std::vector<Util::Vector>();
	bool is_colliding = GJK(simp, _shapeA, _shapeB);
	if (is_colliding) {
		EPA(return_penetration_depth, return_penetration_vector, simp, _shapeA, _shapeB);
		return true;
	}
	else {
		return_penetration_depth = 0.0f;
		return false;
	}
}

