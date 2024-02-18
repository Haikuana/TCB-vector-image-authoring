
#ifndef IMAGE_DOMAIN_H
#define IMAGE_DOMAIN_H

#define DOMAIN_MAXX 0.5

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class ImageDomain
{
private:
	int            _width;
	int            _height;
	int            _channel;
	unsigned char *_data;

	double         _ratio;
	double         _dy;
	double         _pixWidth;

public:
	ImageDomain()
		: _data(NULL), _width(0), _height(0), _channel(0), _ratio(1.0), _dy(DOMAIN_MAXX), _pixWidth(0.0)
	{

	}

	~ImageDomain()
	{
		clear();
	}

	void clear();
	const unsigned char* data() const { return _data; }
	inline int width() const { return _width; }
	inline int height() const { return _height; }
	inline double dx() const { return DOMAIN_MAXX; }
	inline double dy() const { return _dy; }
	inline int channel() const { return _channel; }
	inline int pixel_index(const int i, const int j) const { return j * _width + i; }
	inline double domain_area() const { return 4 * DOMAIN_MAXX * _dy; }
	inline double pixel_area() const { return _pixWidth * _pixWidth; }
	inline double pixel_width() const { return _pixWidth; }
	inline double ratio() const { return _ratio; }

	void set_image(const unsigned char* inData, const int& w, const int& h, const int& c);

	void left_bottom(double *p) const;
	void right_bottom(double *p) const;
	void right_top(double *p) const;
	void left_top(double *p) const;
	void random_point(double *p) const;

	inline double left_x(const int& i) const { return _pixWidth * i - DOMAIN_MAXX; }
	inline double center_x(const int& i) const { return _pixWidth * (i + 0.5) - DOMAIN_MAXX; }
	inline double bottom_y(const int& j) const { return _pixWidth * j - _dy; }
	inline double center_y(const int& j) const { return _pixWidth * (j + 0.5) - _dy; }
	void pixel_corner_coordinate(const int& i, const int& j, double *p) const;
	void pixel_center_coordinate(const int& i, const int& j, double *p) const;
	void pixel_location(const double *p, int &i, int &j) const;

	const unsigned char* pixel_color_pointer(const int& i, const int& j) const;
};

#endif
