#ifndef __LOOP_H__
#define __LOOP_H__

#include "Assistant/data_types.h"
#include "mesh.h"

static double timing_sub_s = 0.0;

typedef std::map<SUB_SIMPLIFY::vertpair, SUB_SIMPLIFY::vertpair> FeaturePair;

namespace SUBDIVISION {

	enum V_type
	{
		SMOOTH_V,
		CORNER_V,//FEATURE CORNER
		CREASE_V_ONE_CORNER_NEIBOR,
		CREASE_V_TWO_CORNER_NEIBOR,
		CREASE_V
	};

	enum E_type
	{
		SMOOTH_E,
		CREASE_E_ONE_CORNER_NEIBOR,
		CREASE_E_OTHER
	};

	class LOOP {
	public:
		LOOP(SUB_SIMPLIFY::MeshSimplify *mesh1,
			SUB_SIMPLIFY::MeshSimplify *mesh2,bool blimit = false);
		~LOOP() {};

		void subdivide();

		void limit_pos();

		inline void set_blimit(bool bc) { bLimitPos = bc; }
		inline void set_subdivided_color(bool bc) { bsubdivided_color = bc; }

		inline void set_feature_line
		(std::vector<std::vector<int>> *in_feature_lines) {feature_lines = in_feature_lines;}

		inline void set_fea_e_pair(FeaturePair fea_e_p) { fea_e_pair = fea_e_p; }
		inline void get_fea_e_pair(FeaturePair &fea_e_p) { fea_e_p = fea_e_pair; }

		inline void get_subdivision_mat(SpMat &mat) { mat = mat_subdivision; }

		inline void set_fea_v_pair(std::map<int, int> *fea_v_p){fea_v_pair = fea_v_p;}

	protected:
		void classify_v(SUB_SIMPLIFY::MeshSimplify *mesh,
			int v, int &type, pair<int,int> &chain2);
		int classify_e(SUB_SIMPLIFY::vertpair);

		float calculateAlpha(int n);
		float calculatelimitW(int n);

		SUB_SIMPLIFY::MeshSimplify *mesh_in;
		SUB_SIMPLIFY::MeshSimplify *mesh_out;

		bool						bsubdivided_color = false;
		bool						bLimitPos;

		std::vector<std::vector<int>>* feature_lines = NULL;
		FeaturePair					fea_e_pair;
		map<int, int>				*fea_v_pair;

		SpMat						mat_subdivision;
	};

}

#endif
