#include "fitting_color.h"

void FittingColor::solve_equation()
{
	mwArray A;
	vector<mwSize> rowIndices;
	vector<mwSize> columnIndices;
	vector<mxDouble> aData;

	for (int k = 0; k < mat_sub.outerSize(); ++k)
	{
		for (SpMat::InnerIterator it(mat_sub, k); it; ++it)
		{
			rowIndices.push_back(it.col()+1);
			columnIndices.push_back(it.row()+1);
			aData.push_back(it.value());
		}
	}
	A = mwArray::NewSparse(rowIndices.size(), rowIndices.data(), columnIndices.size(), columnIndices.data(),
		aData.size(), aData.data(), aData.size());


	vector<mxDouble>				bData;
	for (int i = 0;i<mesh_sub->vertices.size();i++)
	{
		bData.push_back(mesh_sub->vertices[i]->color[0]);
	}
	for (int i = 0; i < mesh_sub->vertices.size(); i++)
	{
		bData.push_back(mesh_sub->vertices[i]->color[1]);
	}
	for (int i = 0; i < mesh_sub->vertices.size(); i++)
	{
		bData.push_back(mesh_sub->vertices[i]->color[2]);
	}
	mwArray b = mwArray(bData.size(), 1, mxDOUBLE_CLASS);
	b.SetData(bData.data(), bData.size());

	std::cout << "MatLab solver beginning\n";
	clock_t start_ = clock();
	mwArray cp, Asample;
	sparse_quad_prog_color(2, cp, Asample, A, b);

	clock_t finish_ = clock();
	std::cout<<"timing: "<<(double(finish_ - start_)) / CLOCKS_PER_SEC;

	int Nc = cp.NumberOfElements();
	double *cps = new mxDouble[Nc];
	cp.GetData(cps, Nc);

	int Np = Nc / 3;
	for (int i = 0;i<Np;i++)
	{
		double r = cps[i];
		double g = cps[i + Np];
		double b = cps[i + Np * 2];
		mesh_simplified->vertices[i]->fitted_color = Eigen::Vector3f(r, g, b);
	}

	std::cout << "number of control mesh p: " << mesh_simplified->vertices.size() << std::endl;
	std::cout << "number of control mesh f: " << mesh_simplified->faces.size() << std::endl;


	int Nax = Asample.NumberOfElements();
	double *AX = new mxDouble[Nax];
	Asample.GetData(AX, Nax);
	int Nsub = Nax / 3;
	for (int i = 0;i<Nsub;i++)
	{
		double r = AX[i];
		double g = AX[i + Nsub];
		double b = AX[i + Nsub * 2];
		mesh_sub->vertices[i]->fitted_color = Eigen::Vector3f(r, g, b);
	}
}
