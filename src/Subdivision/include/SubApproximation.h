#pragma once
#ifndef SubAPPROXIMATION_H
#define SubAPPROXIMATION_H

#include "Corefuncs/ImageApproximation.h"
#include "simplify.h"
#include "loop.h"
#include "fitting_color.h"

#define MAX_DIS_TO_FEATURE_EDGES 1.0

class Sub_Approximation : public Image_Approximation
{
public:
	Sub_Approximation(void) {
		kdTree_ori_image = NULL;
	}
	~Sub_Approximation(void) {
		if (kdTree_ori_image != NULL)
		{
			delete kdTree_ori_image;
			kdTree_ori_image = NULL;
		}
	}

	void					simplify_(double);

	inline SUB_SIMPLIFY::MeshSimplify *			get_control_mesh() { return &mesh_simplified; }
	inline SUB_SIMPLIFY::MeshSimplify *			get_sub_mesh() { return &mesh_sub; }
	//inline std::vector<std::vector<int>> get_fea() { return new_feature; }
	inline std::vector<std::vector<Point_2>> get_infea() { return in_feature; }

	inline FeaturePair get_sub_fea_e_pair() { return sub_fea_e_pair; }
	inline FeaturePair get_simplified_fea_e_pair() { return simpli_fea_e_pair; }

	void					subdivision_once();
	void					subdivision(int times = 2);

	void					cal_target_color();
	void					optimize_color();

	void					mesh_fitting_via_section6();
	void					optimize_knots_triangulation_revised();

private:
	
	void					save_subdivision_mesh(std::string outfile, const SUB_SIMPLIFY::MeshSimplify& mesh);
	void					classify_subdivied_vs(std::vector<bool> &bSafe);
	void					load_image_mesh();

	double						DisPixel = 0.0;

	SUB_SIMPLIFY::MeshSimplify	mesh_image;
	SUB_SIMPLIFY::MeshSimplify	mesh_simplified;
	SUB_SIMPLIFY::MeshSimplify	mesh_sub;

	std::vector<std::vector<int>> new_feature;
	std::vector<std::vector<Point_2>> in_feature;

	ANNkd_tree *				kdTree_ori_image;

	FeaturePair					sub_fea_e_pair;
	FeaturePair					simpli_fea_e_pair;

	map<int, int>				fea_v_pair;

	SpMat						mat_sub;
};

#endif
