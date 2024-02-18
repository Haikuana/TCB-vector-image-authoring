#ifndef __MESH_H__
#define __MESH_H__

#include <Eigen/Dense>
#include <vector>
#include <set>
#include <iostream>
#include <fstream>

namespace SUB_SIMPLIFY {

	struct vertes_type
	{
		bool			is_corner;
		int				bound_type;
		bool			is_fea_corner;
		int				feature_type;
	};

	enum normal_mode {
		NO_NORMALS,
		AVERAGE,
		AREA_WEIGHTS,
		ANGLE_WEIGHTS
	};

	typedef std::pair<int, int> vertpair;
	vertpair makeVertpair(int v1, int v2);

	class vertex {
	public:
		vertex();
		void operator=(const vertex &other);
		int				id;
		bool			added = false;
		bool			bremoved = false;
		Eigen::Vector3f loc_ori;
		Eigen::Vector3f loc_back;
		Eigen::Vector3f loc;
		Eigen::Vector3f color;
		Eigen::Vector3f fitted_color;
		Eigen::Vector3f normal;
		vertes_type		type_back;
		vertes_type		type;
		std::set<int>	neighbors;
	};

	class face {
	public:
		face();

		bool					bshow = true;
		int						id;
		Eigen::Vector3f			normal;
		std::vector<vertex*>	verts;
		std::vector<face*>		neighbors;
		std::vector<vertpair>	connectivity;

		std::vector<vertex*>	verts_back;
		std::vector<vertpair>	connectivity_back;
	};

	class MeshSimplify {
	public:

		std::vector<vertex*> vertices;
		std::vector<face*> faces;
		bool manifold;
		int max_vertex_id;
	};
}

#endif
