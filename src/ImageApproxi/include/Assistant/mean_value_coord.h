#ifndef IMAGE_WARPING
#define IMAGE_WARPING
#include "Assistant/data_types.h"
using namespace std;

class mean_value_warping
{
public:
	mean_value_warping();
	int mean_value_coordinates_with_dir(const vector<Point_2> &cageCoords,
		const Point_2 &queryCoord,
		vector<double> &baryCoords, vector<double> &dirx, vector<double> &diry);
	void do_warp(const vector<Point_2> &cageCoords,const vector<Point_2> warped_cageCoords,
		const vector<Point_2> input_data,vector<Point_2> &warped_data);
	vector<Point_2> cageCoords;
	vector<Point_2> warped_cageCoords;
	vector<Point_2> input_data;
	vector<Point_2> warped_data;
	~mean_value_warping();

private:

};
#endif
