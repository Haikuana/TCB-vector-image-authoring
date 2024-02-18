
#ifndef IMAGE_RASTERIZER_H
#define IMAGE_RASTERIZER_H

#include "OptimizeKnotTri/image_pixelset.h"
#include "OptimizeKnotTri/image_domain.h"

class Rasterizer
{
private:
	ImageDomain *_domain;

public:
	Rasterizer(ImageDomain *d)
		:_domain(NULL)
	{
		_domain = d;
	}

	~Rasterizer()
	{
		_domain = NULL;
	}

	void rasterize(const double *polygon, const int vnb, PixelSet &pixels)
	{
		if (_domain == NULL)
			return;

		if (vnb < 3)
			return;

		// declaration
		int* x_left = NULL;
		int* x_right = NULL;
		std::vector<int> x_;
		std::vector<int> y_;

		// initialization
		x_left = new int[_domain->height()];
		x_right = new int[_domain->height()];
		x_.clear();
		y_.clear();
		for (int v = 0; v < vnb; v++)
		{
			int i = -1, j = -1;
			_domain->pixel_location(&polygon[2 * v], i, j);

			x_.push_back(i);
			y_.push_back(j);
		}

		int ymin = 32767;
		int ymax = -1;

		rasterize(x_, y_, ymin, ymax, x_left, x_right);

		pixels.set_bottom(ymin);
		pixels.set_top(ymax);
		pixels.left().clear();
		pixels.right().clear();

		for (int i = ymin; i <= ymax; i++)
		{
			pixels.left().push_back(x_left[i]);
			pixels.right().push_back(x_right[i]);
		}

		delete[] x_left;
		x_left = NULL;
		delete[] x_right;
		x_right = NULL;
		x_.clear();
		y_.clear();
	}

protected:
	void rasterize(std::vector<int> &x_, std::vector<int> &y_, int &ymin, int &ymax, int *x_left, int *x_right)
	{
		int N = (int)x_.size();

		for (int i = 0; i < N; ++i)
		{
			ymin = std::min(ymin, y_[i]);
			ymax = std::max(ymax, y_[i]);
		}

		for (int i = ymin; i <= ymax; ++i)
		{
			x_left[i] = _domain->width() - 1;
			x_right[i] = 0;
		}

		int signed_area = (x_[1] - x_[0]) * (y_[2] - y_[0]) - (x_[2] - x_[0]) * (y_[1] - y_[0]);
		bool ccw = (signed_area < 0);

		for (int i = 0; i < N; ++i)
		{
			int j = (i + 1) % N;
			int x1 = x_[i];
			int y1 = y_[i];
			int x2 = x_[j];
			int y2 = y_[j];
			if (y1 == y2)
			{
				if (x2 > x1)
				{
					x_left[y1] = std::min(x_left[y1], x1);
					x_right[y1] = std::max(x_right[y1], x2);
				}
				else
				{
					x_left[y1] = std::min(x_left[y1], x2);
					x_right[y1] = std::max(x_right[y1], x1);
				}
				continue;
			}

			bool is_left = (y2 < y1) ^ ccw;

			// Bresenham algo.
			int dx = x2 - x1;
			int dy = y2 - y1;
			int sx = dx > 0 ? 1 : -1;
			int sy = dy > 0 ? 1 : -1;
			dx *= sx;
			dy *= sy;
			int x = x1;
			int y = y1;

			int* line_x = is_left ? x_left : x_right;
			if ((is_left && sx > 0) || (!is_left && sx < 0))
				line_x[y] = x;

			int e = dy - dx;
			while ((sy > 0 && y < y2) || (sy < 0 && y > y2))
			{
				while (e < 0 || (e == 0 && ((is_left && sx > 0) || (!is_left && sx < 0))))
				{
					x += sx;
					e += 2 * dy;
				}
				if ((!is_left && sx > 0) || (is_left && sx < 0))
					line_x[y] = x;
				y += sy;
				e -= 2 * dx;
				if ((is_left && sx > 0) || (!is_left && sx < 0))
					line_x[y] = x;
			}
			if ((!is_left && sx > 0) || (is_left && sx < 0))
				line_x[y2] = x2;
		}
	}
};

#endif
