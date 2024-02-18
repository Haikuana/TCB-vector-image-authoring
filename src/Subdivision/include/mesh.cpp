#include "mesh.h"

using namespace std;
using namespace Eigen;

namespace SUB_SIMPLIFY {

	vertex::vertex() {
		loc.setZero();
		loc_back.setZero();
		normal.setZero();
	}

	void vertex::operator=(const vertex &other) {
		loc = other.loc;
		id = other.id;
		type = other.type;
	}

	face::face() {
		verts = vector<vertex*>(3);
		verts_back = vector<vertex*>(3);
	}

}