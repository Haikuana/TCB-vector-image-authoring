#ifndef __SSSSOBJPARSE_H__
#define __SSSSOBJPARSE_H__

#include "mesh.h"
#include "Assistant/data_types.h"

namespace SUB_SIMPLIFY {

	void loadObjFile(std::string input, MeshSimplify& mesh, 
		std::vector<std::vector<int>> feature_lines, std::map<int, int> &fea_pairs);
	void saveObjFile(std::string output, const MeshSimplify& mesh);

	void loadFeatureFile(std::string infile, std::vector<std::vector<int>> &feature_lines);
}
#endif
