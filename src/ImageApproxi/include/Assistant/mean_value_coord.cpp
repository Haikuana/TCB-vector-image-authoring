#include "mean_value_coord.h"
using namespace std;

mean_value_warping::mean_value_warping()
{
	
	;
}
int mean_value_warping::mean_value_coordinates_with_dir(const vector<Point_2> &cageCoords,
	const Point_2 &queryCoord,
	vector<double> &baryCoords, vector<double> &dirx, vector<double> &diry)
{
	//////////////////////////////////////////////////////////////////////////////////
	// Input :
	//      1. cageCoords :  Coordinates of closed polygon in the Counter
	//                       clockwise direction. The input is not tested inside.
	//
	//       2. queryCoord:   the xyCoords of the query Point_2
	// Output:
	//       1:  baryCoords: baryCentric Coords of the query Point_2.
	//       2:  dir_x: dir of baryCoords with respect to x
	//       3:  dir_y: dir of baryCoords with repsect to y
	//
	// Reference: Mean Value Coordinates for Arbitrary Planar Polygons:
	//            Kai Hormann and Michael Floater;
	// Written by:
	//            Juan Cao
	//            Xiamen University
	//            18th March, 2013.
	/////////////////////////////////////////////////////////////////////////////////

	int nSize = cageCoords.size();
	assert(nSize);

	double dx, dy;

	vector<Point_2>  s(nSize);
	vector<Point_2> xxi(nSize);

	for (int i = 0; i < nSize; i++)
	{
		dx = cageCoords[i][0] - queryCoord[0];
		dy = cageCoords[i][1] - queryCoord[1];
		s[i] = Point_2(dx, dy);
		xxi[i] = Point_2(cageCoords[(i + 1) % nSize][0] - cageCoords[i][0], cageCoords[(i + 1) % nSize][1] - cageCoords[i][1]);
	}

	baryCoords.resize(nSize);
	dirx.resize(nSize);
	diry.resize(nSize);

	for (int i = 0; i < nSize; i++)
		baryCoords[i] = 0.0;

	int ip, im;      // (i+1) and (i-1)
	double ri, rp, Ai, Di, dl, mu;  // Distance
	double eps = 1.0e-10;//10.0*std::numeric_limits<double>::min();

	bool is_querypoints_on_edge = false;

	int on_jth_edge = 0;
	bool is_perturbed = false;
	double perturb_factor = 1.0e-16;


	double Aix, Aiy;
	vector<double> tanalpha(nSize); // tan(alpha/2)
	vector<double> tanalphax(nSize);
	vector<double> tanalphay(nSize);

	double rix, riy, rpx, rpy, Dix, Diy; ///derivative of all variables

	for (int i = 0; i < nSize; i++)
	{
		ip = (i + 1) % nSize;
		im = (nSize - 1 + i) % nSize;
		ri = sqrt(s[i][0] * s[i][0] + s[i][1] * s[i][1]);
		rix = -(s[i][0]) / ri;
		riy = -(s[i][1]) / ri;
		rp = sqrt(s[ip][0] * s[ip][0] + s[ip][1] * s[ip][1]);
		rpx = -(s[ip][0]) / rp;
		rpy = -(s[ip][1]) / rp;

		Ai = 0.5*(s[i][0] * s[ip][1] - s[ip][0] * s[i][1]);
		Aix = 0.5*(s[i][1] - s[ip][1]);
		Aiy = 0.5*(s[ip][0] - s[i][0]);

		Di = s[ip][0] * s[i][0] + s[ip][1] * s[i][1];
		Dix = -(s[ip][0] + s[i][0]);
		Diy = -(s[ip][1] + s[i][1]);

		if (fabs(Ai) > fabs(ri*rp + Di)) // the following two formulae are unstable on the
			// points near the boundary or on the extension of cage edges
			//so have to used the other one in corresponding case
		{
			tanalpha[i] = (ri*rp - Di) / (2.0*Ai);
			tanalphax[i] = (rix*rp + ri*rpx - Dix) / (2.0*Ai) - Aix*(ri*rp - Di) / (2.0*Ai*Ai);
			tanalphay[i] = (riy*rp + ri*rpy - Diy) / (2.0*Ai) - Aiy*(ri*rp - Di) / (2.0*Ai*Ai);
		}
		else
		{
			tanalpha[i] = 2.0*Ai / (ri*rp + Di);
			tanalphax[i] = 2.0*Aix / (ri*rp + Di) - 2 * Ai*(rix*rp + ri*rpx + Dix) / pow(ri*rp + Di, 2);
			tanalphay[i] = 2.0*Aiy / (ri*rp + Di) - 2 * Ai*(riy*rp + ri*rpy + Diy) / pow(ri*rp + Di, 2);
		}


	}

	double dx_, dy_, dl_;
	// First check if any coordinates close to the cage point or
	// lie on the cage boundary. These are special cases, should use a modified formulation.


	for (int i = 0; i < nSize; i++)
	{
		ip = (i + 1) % nSize;
		ri = sqrt(s[i][0] * s[i][0] + s[i][1] * s[i][1]);
		Ai = 0.5*(s[i][0] * s[ip][1] - s[ip][0] * s[i][1]);
		Aix = 0.5*(s[i][1] - s[ip][1]);
		Aiy = 0.5*(s[ip][0] - s[i][0]);
		Di = s[ip][0] * s[i][0] + s[ip][1] * s[i][1];
		if (ri <= eps)
		{
			baryCoords[i] = 1.0;
			baryCoords[ip] = 0.0;

			dx_ = cageCoords[ip][0] - cageCoords[i][0];
			dy_ = cageCoords[ip][1] - cageCoords[i][1];
			dl_ = sqrt(dx_*dx_ + dy_*dy_);
			assert(dl_ > eps);
			dx = queryCoord[0] - cageCoords[ip][0];
			dy = queryCoord[1] - cageCoords[ip][1];
			dl = sqrt(dx*dx + dy*dy);
			mu = dl / dl_;

			//	assert( mu >= 0.0 && mu <= 1.0);

			if (mu<0.0)
				mu = 0.0;
			else if (mu > 1.0)
				mu = 1.0;
			else
				mu = mu;
			baryCoords[i] = mu;
			baryCoords[ip] = 1 - mu;

			dirx[i] = dx / (dl*dl_);
			diry[i] = dy / (dl*dl_);
			dirx[ip] = -dx / (dl*dl_);
			diry[ip] = -dy / (dl*dl_);

			return 0;
		}
		else if (fabs(Ai) <= 1.0e-10 && Di < 0.0)
		{
			dx_ = cageCoords[ip][0] - cageCoords[i][0];
			dy_ = cageCoords[ip][1] - cageCoords[i][1];
			dl_ = sqrt(dx_*dx_ + dy_*dy_);
			assert(dl_ > eps);
			dx = queryCoord[0] - cageCoords[i][0];
			dy = queryCoord[1] - cageCoords[i][1];
			dl = sqrt(dx*dx + dy*dy);

			mu = dl / dl_;
			assert(mu >= 0.0 && mu <= 1.0);
			baryCoords[i] = 1.0 - mu;
			baryCoords[ip] = mu;

			/////////////if Ai == 0, v is on the edge of v_iv_i+1
			/////////////then we calculate the derivative using a modified formular,
			/////////////say, omiga_j is replaced by omiga_j* Ai/2////////

			vector<double> AP(nSize);
			vector<double> AP_dirx(nSize);
			vector<double> AP_diry(nSize);
			vector<double> A(nSize);
			vector<double> B(nSize);
			vector<double> B_dirx(nSize);
			vector<double> B_diry(nSize);
			vector<double> r(nSize);
			vector<double> r_dirx(nSize);
			vector<double> r_diry(nSize);


			double rj, rjx, rjy, rpx, rpy, Aj, Ajx, Ajy, Dj, Djx, Djy; ///derivative of all variables
			int jp, jm;

			for (int j = 0; j < nSize; j++)
			{
				jp = (j + 1) % nSize; // j+1
				jm = (j + nSize - 1) % nSize;

				A[j] = 0.5*(xxi[jm][0] * xxi[j][1] - xxi[jm][1] * xxi[j][0]); // area of triangle v_j-1,v_j,v_j+1
				AP[j] = 0.5*(s[j][0] * s[jp][1] - s[jp][0] * s[j][1]); // area of triangle v_j,v_j+1,p
				AP_dirx[j] = 0.5*(s[j][1] - s[jp][1]);
				AP_diry[j] = 0.5*(s[jp][0] - s[j][0]);

				r[j] = sqrt(s[j][0] * s[j][0] + s[j][1] * s[j][1]);
				r_dirx[j] = -(s[j][0]) / r[j];
				r_diry[j] = -(s[j][1]) / r[j];
			}

			for (int j = 0; j < nSize; j++)
			{
				jp = (j + 1) % nSize; // j+1
				jm = (j + nSize - 1) % nSize;

				B[j] = -A[j] + AP[jm] + AP[j];  // area of triangle v, v_j-1,v_j+1
				B_dirx[j] = AP_dirx[jm] + AP_dirx[j];
				B_diry[j] = AP_diry[jm] + AP_diry[j];
			}


			// Equation #11, from the paper
			double wj, wsum = 0.0;
			double wjx, wjy;
			double wsum_dirx = 0.0;
			double wsum_diry = 0.0;
			for (int j = 0; j < nSize; j++)
			{
				jm = (nSize - 1 + j) % nSize;
				jp = (j + 1) % nSize;


				if (j == i)
				{
					baryCoords[j] = (-r[j] * B[j] + r[jp] * AP[jm]) / AP[jm];
					dirx[j] = (r[jm] * AP_dirx[j]) / AP[jm] + r_dirx[jp] - (r_dirx[j] * B[j] + r[j] * B_dirx[j]) / AP[jm] + r[j] * B[j] * AP_dirx[jm] / (AP[jm] * AP[jm]);
					diry[j] = (r[jm] * AP_diry[j]) / AP[jm] + r_diry[jp] - (r_diry[j] * B[j] + r[j] * B_diry[j]) / AP[jm] + r[j] * B[j] * AP_diry[jm] / (AP[jm] * AP[jm]);


				}
				else if (j == (i + 1) % nSize)
				{
					baryCoords[j] = (-r[j] * B[j]) / AP[j] + r[jm];
					dirx[j] = r_dirx[jm] - (r_dirx[j] * B[j] + r[j] * B_dirx[j]) / AP[j] + (r[j] * B[j] - r[jp] * AP[jm])*AP_dirx[j] / (AP[j] * AP[j]) + (r[jp] * AP_dirx[jm]) / AP[j];
					diry[j] = r_diry[jm] - (r_diry[j] * B[j] + r[j] * B_diry[j]) / AP[j] + (r[j] * B[j] - r[jp] * AP[jm])*AP_diry[j] / (AP[j] * AP[j]) + (r[jp] * AP_diry[jm]) / AP[j];
				}
				else
				{
					if (fabs(AP[jm]) > 1.0e-12 && fabs(AP[j]) > 1.0e-12) // query point is not on the intersection of 
					{
						baryCoords[j] = 0;
						dirx[j] = (r[jm] * AP[j] - r[j] * B[j] + r[jp] * AP[jm])*AP_dirx[i] / (AP[jm] * AP[j]);
						diry[j] = (r[jm] * AP[j] - r[j] * B[j] + r[jp] * AP[jm])*AP_diry[i] / (AP[jm] * AP[j]);
					}
					else
					{
						double wi, wix, wiy;
						im = (nSize - 1 + j) % nSize;
						ri = sqrt(s[j][0] * s[j][0] + s[j][1] * s[j][1]);
						rix = -(s[j][0]) / ri;
						riy = -(s[j][1]) / ri;

						wi = Ai*2.0*(tanalpha[j] + tanalpha[im]) / ri;
						wix = (2.0*(tanalphax[j] + tanalphax[im]) / ri - wi*rix / ri)*Ai + Aix*2.0*(tanalpha[j] + tanalpha[im]) / ri;
						wiy = (2.0*(tanalphay[j] + tanalphay[im]) / ri - wi*riy / ri)*Ai + Aiy*2.0*(tanalpha[j] + tanalpha[im]) / ri;

						baryCoords[j] = wi;
						dirx[j] = wix;
						diry[j] = wiy;

					}

				}

				wsum += baryCoords[j];
				wsum_dirx += dirx[j];
				wsum_diry += diry[j];

			}


			if (fabs(wsum) > 0.0)
			{
				for (int j = 0; j < nSize; j++)
				{

					dirx[j] = dirx[j] / wsum - baryCoords[j] * wsum_dirx / (wsum*wsum);
					diry[j] = diry[j] / wsum - baryCoords[j] * wsum_diry / (wsum*wsum);
					baryCoords[j] /= wsum;

				}
			}
			return 0;
		}

	}

	// Equation #11, from the paper
	double wi, wsum = 0.0;
	double wix, wiy;
	double wsum_dirx = 0.0;
	double wsum_diry = 0.0;
	for (int i = 0; i < nSize; i++)
	{
		im = (nSize - 1 + i) % nSize;
		ri = sqrt(s[i][0] * s[i][0] + s[i][1] * s[i][1]);
		rix = -(s[i][0]) / ri;
		riy = -(s[i][1]) / ri;

		wi = 2.0*(tanalpha[i] + tanalpha[im]) / ri;
		wix = 2.0*(tanalphax[i] + tanalphax[im]) / ri - wi*rix / ri;
		wiy = 2.0*(tanalphay[i] + tanalphay[im]) / ri - wi*riy / ri;
		wsum += wi;
		wsum_dirx += wix;
		wsum_diry += wiy;

		baryCoords[i] = wi;
		dirx[i] = wix;
		diry[i] = wiy;

	}

	if (fabs(wsum) > 0.0)
	{
		for (int i = 0; i < nSize; i++)
		{

			dirx[i] = dirx[i] / wsum - baryCoords[i] * wsum_dirx / (wsum*wsum);
			diry[i] = diry[i] / wsum - baryCoords[i] * wsum_diry / (wsum*wsum);
			baryCoords[i] /= wsum;
		}
	}

	return 0;

}
void mean_value_warping::do_warp(const vector<Point_2> &cageCoords, const vector<Point_2> warped_cageCoords,
	const vector<Point_2> input_data, vector<Point_2> &warped_data)
{
	warped_data.resize(input_data.size());
	for (int i = 0; i < input_data.size(); i++)
	{
		vector<double> baryCoords, dirx, diry;
		mean_value_coordinates_with_dir(cageCoords, input_data[i], baryCoords, dirx, diry);
		double tmp_x = 0, tmp_y = 0;
		for (int j = 0; j < baryCoords.size(); j++)
		{
			tmp_x += baryCoords[j] * warped_cageCoords[j].x();
			tmp_y += baryCoords[j] * warped_cageCoords[j].y();
		}
		warped_data[i] = Point_2(tmp_x, tmp_y);
	}
}
mean_value_warping::~mean_value_warping()
{
	;
}