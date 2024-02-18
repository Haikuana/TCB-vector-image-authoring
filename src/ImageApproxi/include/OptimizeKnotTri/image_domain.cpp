
#include <algorithm>
#include "OptimizeKnotTri/image_domain.h"

void ImageDomain::clear()
{
	if (_data)
	{
		delete[] _data;
		_data = 0;
	}

	_width = 0;
	_height = 0;
	_channel = 0;

	_ratio = 1.0;
	_dy = DOMAIN_MAXX;
}

void ImageDomain::set_image(const unsigned char* inData, const int& w, const int& h, const int& c)
{
	if (w == 0 || h == 0 || c == 0)
	{
		printf("%s: image size illegal!\n", __FUNCTION__);
		return;
	}

	clear();

	_width = w;
	_height = h;
	_channel = c;
	_ratio = double(_height) / double(_width);
	_dy = _ratio * DOMAIN_MAXX;
	_pixWidth = 2.0 * DOMAIN_MAXX / double(_width);

	_data = new unsigned char[w * h * c];
	memcpy(_data, inData, sizeof(unsigned char) * w * h * c);
}

void ImageDomain::left_bottom(double *p) const
{
	p[0] = -DOMAIN_MAXX;
	p[1] = -_dy;
}

void ImageDomain::right_bottom(double *p) const
{
	p[0] = DOMAIN_MAXX;
	p[1] = -_dy;
}

void ImageDomain::right_top(double *p) const
{
	p[0] = DOMAIN_MAXX;
	p[1] = _dy;
}

void ImageDomain::left_top(double *p) const
{
	p[0] = -DOMAIN_MAXX;
	p[1] = _dy;
}

void ImageDomain::random_point(double *p) const
{
	double a = double(rand()) / RAND_MAX;
	double b = double(rand()) / RAND_MAX;

	p[0] = a * DOMAIN_MAXX * 2 - DOMAIN_MAXX;
	p[1] = b * _dy * 2 - _dy;
}


void ImageDomain::pixel_corner_coordinate(const int& i, const int& j, double *p) const
{
	p[0] = left_x(i);
	p[1] = bottom_y(j);
}

void ImageDomain::pixel_center_coordinate(const int& i, const int& j, double *p) const
{
	p[0] = center_x(i);
	p[1] = center_y(j);
}

void ImageDomain::pixel_location(const double *p, int &i, int &j) const
{
	double x = 0.5 * (p[0] + DOMAIN_MAXX) / DOMAIN_MAXX;
	double y = 0.5 * (_dy + p[1]) / _dy;

	x = (std::max)(0.0, x);
	x = (std::min)(1.0, x);
	y = (std::max)(0.0, y);
	y = (std::min)(1.0, y);

	int w = _width;
	int h = _height;

	i = (int)floor(x * double(w));
	j = (int)floor(y * double(h));

	if (i == w) i--;
	if (j == h) j--;
}

const unsigned char* ImageDomain::pixel_color_pointer(const int& i, const int& j) const
{
	int pixID = pixel_index(i, j);
	return &_data[_channel * pixID];
}
