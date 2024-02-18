
#ifndef IMAGE_POLYNOMIAL_H
#define IMAGE_POLYNOMIAL_H

#include <vector>
#include <math.h>
#include <Eigen/Eigen>

#include "OptimizeKnotTri/image_pixelset.h"
#include "OptimizeKnotTri/image_domain.h"

class Polynomial
{
public:
	enum { CONSTANT_APPROXIMATION = 0, LINEAR_APPROXIMATION = 1, QUADRATIC_APPROXIMATION = 2 };

private:
	double _coeff[3][6];
	int _degree;

	ImageDomain *_domain;

public:
	Polynomial(ImageDomain *domain = NULL, const int d = LINEAR_APPROXIMATION)
		: _domain(domain), _degree(-1)
	{
		set_degree(d);
	}

	~Polynomial()
	{
		clear();
		_domain = NULL;
	}

	void clear()
	{
		
	}

	Polynomial& operator= (const Polynomial &rhs)
	{
		_domain = rhs._domain;
		set_degree(rhs._degree);
		memcpy(_coeff, rhs._coeff, 18 * sizeof(double));

		return *this;
	}

	int degree() const
	{
		return _degree;
	}

	void set_degree(const int d)
	{
		if (d == _degree)
			return;

		_degree = d;
	}

	void init()
	{
		memset(_coeff, 0, 18 * sizeof(double));
	}

	void compute_factors(const PixelSet* pixels)
	{
		if (pixels == NULL || _domain == NULL)
		{
			printf("%s: null\n", __FUNCTION__);
			return;
		}

		init();

		switch (_degree)
		{
		case LINEAR_APPROXIMATION:
			compute_linear_factors(pixels);
			break;
		case QUADRATIC_APPROXIMATION:
			compute_quadratic_factors(pixels);
			break;
		default:
			compute_constant_factors(pixels);
			break;
		}
	}

	double evaluate(const int channel, const double x, const double y) const
	{
		double result = 0.0;

		switch (_degree)
		{
		case CONSTANT_APPROXIMATION:
			result = _coeff[channel][0];
			break;
		case LINEAR_APPROXIMATION:
			result = _coeff[channel][0] * x + _coeff[channel][1] * y + _coeff[channel][2];
			break;
		case QUADRATIC_APPROXIMATION:
			result = _coeff[channel][0] * x * x + _coeff[channel][1] * x * y + _coeff[channel][2] * y * y 
				+ _coeff[channel][3] * x + _coeff[channel][4] * y + _coeff[channel][5];
			break;
		}

		return result;
	}
	double evaluate(const int channel, const double* xy) const
	{
		return evaluate(channel, xy[0], xy[1]);
	}

	double compute_energy(const PixelSet* pixels, const int Lp = 2) const
	{
		if (pixels == NULL || _domain == NULL)
		{
			printf("%s: null\n", __FUNCTION__);
			return 0.0;
		}

		double result = 0.0;
		for (int j = pixels->bottom(); j <= pixels->top(); j++)
		{
			double y = _domain->center_y(j);
			for (int i = pixels->left(j); i <= pixels->right(j); i++)
			{
				double x = _domain->center_x(i);
				const unsigned char* pixColors = _domain->pixel_color_pointer(i, j);
				
				double tempEnergy = 0.0;
				for (int c = 0; c < _domain->channel(); c++)
				{
					double approxVal = evaluate(c, x, y);
					double pixVal = (double)pixColors[c];
					double absError = fabs(pixVal - approxVal);
					tempEnergy += std::pow(absError, Lp);
				}

				result += tempEnergy;
			}
		}

		return result;
	}

protected:
	void compute_constant_factors(const PixelSet* pixels)
	{
		double sumVal[3] = {0.0, 0.0, 0.0};
		
		int sumnb = 0;
		for (int j = pixels->bottom(); j <= pixels->top(); j++)
		{
			for (int i = pixels->left(j); i <= pixels->right(j); i++)
			{
				const unsigned char* pixColors = _domain->pixel_color_pointer(i, j);

				for (int c = 0; c < _domain->channel(); c++)
				{
					sumVal[c] += pixColors[c];
				}

				sumnb++;
			}
		}

		if (sumnb > 0)
		{
			for (int c = 0; c < _domain->channel(); c++)
			{
				_coeff[c][0] = sumVal[c] / sumnb;
			}
		}
	}

	void compute_linear_factors(const PixelSet* pixels)
	{
		Eigen::Matrix3d matA[3];
		Eigen::Vector3d vecB[3];

		for (int c = 0; c < _domain->channel(); c++)
		{
			matA[c].setZero();
			vecB[c].setZero();
		}

		double temp[3];
		double pixArea = _domain->pixel_area();
		for (int j = pixels->bottom(); j <= pixels->top(); j++)
		{
			double y = _domain->center_y(j);
			for (int i = pixels->left(j); i <= pixels->right(j); i++)
			{
				double x = _domain->center_x(i);
				const unsigned char* pixColors = _domain->pixel_color_pointer(i, j);
				
				temp[2] = pixArea;
				temp[0] = x * temp[2];
				temp[1] = y * temp[2];

				for (int c = 0; c < _domain->channel(); c++)
				{
					for (int k = 0; k < 3; k++)
					{
						vecB[c](k) += pixColors[c] * temp[k];

						matA[c](0, k) += x * temp[k];
						matA[c](1, k) += y * temp[k];
						matA[c](2, k) += temp[k];
					}
				}
			}
		}

		for (int c = 0; c < _domain->channel(); c++)
		{
			if (matA[c].determinant() != 0)
			{
				Eigen::Vector3d vecX = matA[c].colPivHouseholderQr().solve(vecB[c]);
				_coeff[c][0] = vecX(0);
				_coeff[c][1] = vecX(1);
				_coeff[c][2] = vecX(2);
			}
			else
			{
				_coeff[c][0] = 0.0;
				_coeff[c][1] = 0.0;
				_coeff[c][2] = 0.0;
				if (matA[c](2, 2) != 0.0)
				{
					//printf("%s: linear back to constant, channel = %d\n", __FUNCTION__, c);
					_coeff[c][2] = vecB[c](2) / matA[c](2, 2);
				}
			}
		}
	}

	void compute_quadratic_factors(const PixelSet* pixels)
	{
		Eigen::MatrixXd matA[3];
		Eigen::VectorXd vecB[3];

		for (int c = 0; c < _domain->channel(); c++)
		{
			matA[c] = Eigen::MatrixXd::Zero(6, 6);
			vecB[c] = Eigen::VectorXd::Zero(6);
		}

		double temp[6];
		double pixArea = _domain->pixel_area();
		for (int j = pixels->bottom(); j <= pixels->top(); j++)
		{
			double y = _domain->center_y(j);
			for (int i = pixels->left(j); i <= pixels->right(j); i++)
			{
				double x = _domain->center_x(i);
				const unsigned char* pixColors = _domain->pixel_color_pointer(i, j);
				
				temp[5] = pixArea;
				temp[0] = x * x * temp[5];
				temp[1] = x * y * temp[5];
				temp[2] = y * y * temp[5];
				temp[3] = x * temp[5];
				temp[4] = y * temp[5];

				for (int c = 0; c < _domain->channel(); c++)
				{
					for (int k = 0; k < 6; k++)
					{
						vecB[c](k) += pixColors[c] * temp[k];

						matA[c](0, k) += x * x * temp[k];
						matA[c](1, k) += x * y * temp[k];
						matA[c](2, k) += y * y * temp[k];
						matA[c](3, k) += x * temp[k];
						matA[c](4, k) += y * temp[k];
						matA[c](5, k) += temp[k];
					}
				}
			}
		}

		for (int c = 0; c < _domain->channel(); c++)
		{
			if (matA[c].determinant() != 0)
			{
				Eigen::VectorXd vecX = matA[c].colPivHouseholderQr().solve(vecB[c]);
				for (int k = 0; k < 6; k++)
				{
					_coeff[c][k] = vecX(k);
				}
			}
			else
			{
				_coeff[c][0] = 0.0;
				_coeff[c][1] = 0.0;
				_coeff[c][2] = 0.0;

				Eigen::Matrix3d matAN;
				matAN(0, 0) = matA[c](0, 5);
				matAN(0, 1) = matA[c](1, 5);
				matAN(0, 2) = matA[c](3, 5);
				matAN(1, 1) = matA[c](2, 5);
				matAN(0, 2) = matA[c](4, 5);
				matAN(2, 2) = matA[c](5, 5);
				matAN(1, 0) = matAN(0, 1);
				matAN(2, 0) = matAN(0, 2);
				matAN(2, 1) = matAN(1, 2);

				if (matAN.determinant() != 0)
				{
					//printf("%s: quadratic back to linear, channel = %d\n", __FUNCTION__, c);
					Eigen::Vector3d vecBN, vecXN;
					for (int k = 0; k < 3; k++)
						vecBN(k) = vecB[c](k + 3);

					vecXN = matAN.colPivHouseholderQr().solve(vecBN);

					for (int k = 0; k < 3; k++)
						_coeff[c][k + 3] = vecXN(k);
				}
				else
				{
					_coeff[c][3] = 0.0;
					_coeff[c][4] = 0.0;
					_coeff[c][5] = 0.0;
					if (matA[c](5, 5) != 0.0)
					{
						//printf("%s: quadratic back to constant, channel = %d\n", __FUNCTION__, c);
						_coeff[c][5] = vecB[c](5) / matA[c](5, 5);
					}
				}				
			}
		}
	}
};

#endif