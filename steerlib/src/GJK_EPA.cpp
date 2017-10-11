#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//implementation of GJK algorithm, return true and simplex is not null or false and simplex is nulls
static bool GJK(std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
	return false;
}

//implementation of EPA algorithm, it is given that GJK has concluded that the shapes are intersecting
static void EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, std::vector<Util::Vector>& simplex,
	const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {

}
//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simp = std::vector<Util::Vector>();
	bool is_colliding = GJK(simp, _shapeA, _shapeB);
	if (is_colliding) {
		EPA(return_penetration_depth, return_penetration_vector, simp, _shapeA, _shapeB);
	}
	else {
		return_penetration_depth = 0.0f;
		return false;
	}
}

