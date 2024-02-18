#ifndef __SIMPLIFY_H__
#define __SIMPLIFY_H__

#include "mesh.h"
#include "objparse.h"

#define DIS_FEATURE_POLY 1.0
#define MIN_NUMBER_OF_FEAPOLY 3

namespace SUB_SIMPLIFY {

	void simplifyMesh(MeshSimplify &mesh, float factor);

	void simplifyTest();

	void simplify(std::string infile_mesh, std::string infile_feature,
		MeshSimplify &meshout, double level_);

	void get_new_featurelines(std::vector<std::vector<int>> &new_feature);

	void get_ori_featurelines(std::vector<std::vector<Point_2>> &feature);

	void get_feature_e_pairs(std::map<vertpair, vertpair> &fea_e_pairs);

	void get_feature_v_pairs(std::map<int, int> &fea_v_pa);
}
#endif
