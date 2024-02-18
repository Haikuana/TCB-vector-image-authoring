#pragma once
#ifndef MARKING_H
#define MARKING_H

#include "Assistant/data_types.h"
#include "Corefuncs/auxfunc.h"
#include "Corefuncs/ImageApproximation.h"

#include "loop.h"
#include "objparse.h"

enum Screen_Mode
{
	BY_DOMIAN, BY_CP
};
#define SCREEN_FEATURE_PLOY_MODE BY_CP//domain 1; cp 0;

namespace EDITGUI {

	enum WindowType
	{
		OPERATION_W, REFERENCE_W, VECIMAGE_W
	};

	enum PickMode
	{
		NONE, SELFPOLYGON, BRUSH, FEATURE, SQUARE,BACK_UP,
		COLOR,RED_CHANNEL, GREEN_CHANNEL, BLUE_CHANNEL,COLOR_BLOCK, COLOR_SLIDER,
		ROTATE, SCALE, TRANSLATION,SINGLE_CP,CWARP_CWROTATE,
		CWARP_CCWROTATE, CWARP_ROOMIN, CWARP_ROOMOUT, CWARP_TRANS,
		INSERT_P,INSERT_NORMAL_E,INSERT_CREASE_E,INSERT_CREASE_SHAPE
	};

	class VecEditing :public QObject
	{
		Q_OBJECT
	public:
		VecEditing();
		~VecEditing();

		bool load_editing_data(QString filename);
		bool load_authoring_data(QString filename);
		bool convert2concise(QString filename);
		bool save_editing_data(QString *filename = NULL);
		bool save_authoring_data(QString* filename = NULL);

		void update_authoring_data();//requirements: given feature lines and domain mesh
		void split_mesh_with_feature(CDT& domain_, AuthoringCM& cm_, ACPs& cps_new);
		void subdivision(int times = 2);

		bool is_editing() { return operation_mode; }

		AuthoringCM* get_authoring_mesh() { return &authorcm; }
		CDT* get_domain_mesh() { return &domain_mesh; }
		ACPs* get_authoring_cps() { return &CPs_new; }
		EditCM* get_control_mesh() { return &cm; }
		//CMInfo* get_vec_image() { return &fm; }
		CPs*	get_cpsP() { return &cps_seqs; }
		CPs&	get_cps() { return cps_seqs; }
		void get_vec_image(RGBPoint** veci, int& size_) {
			*veci = render_vecimage;
			if (operation_mode == 1)
				size_ = fm.faces.size() * 3;
			else
				size_ = mesh_sub.faces.size() * 3;
		}
		float *get_edit_color() { return edit_color; }
		float *get_edit_color_backg() { return edit_color_backg; }

		float* get_authoring_right_color() { return authoring_right_color; }
		float* get_authoring_left_color() { return authoring_left_color; }

		Point_2 get_image_size() { return Point_2(image_wid_, image_hei_); }

		void set_edit_data(set<int>& edit_data_in);
		void interpolated_vecimage(int internal_);
		void update_edited_vecimage(const set<int> &edit_data);

		//get and set model view
		void	GetCameraPosAndOrientation(vector<vector<double>> &pos_orient);
		void	SetCameraPosAndOrientation(vector<vector<double>> pos_orientt);
		bool	&is_fixed_model_view();

	protected:
		bool	generate_output_directory(QString allDir);

	private:
		bool						operation_mode = 1;//0-authoring; 1-editing;

		QString						infilename;

		double						image_wid_;
		double						image_hei_;

		//editing
		EditFM						fm;
		RGBPoint*					render_vecimage;
		EditCM						cm;
		CPs							cps_seqs;

		//authoring
		CDT							domain_mesh;
		AuthoringCM					authorcm;
		ACPs						CPs_new;
		set<int>					edit_data;
		//subdivision
		SUB_SIMPLIFY::MeshSimplify	mesh_sub;
		FeaturePair					fea_e_pair;
		map<int, int>				fea_v_pair;

		float						edit_color[3];
		float						edit_color_backg[3];

		float						authoring_right_color[3];
		float						authoring_left_color[3];

		//model view
		vector<vector<double>>		model_position_orientation;
		bool						is_fixed_modelview;
	};

}

#endif // !MARKING_H
