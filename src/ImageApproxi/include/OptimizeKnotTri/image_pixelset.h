
#ifndef IMAGE_PIXELSET_H
#define IMAGE_PIXELSET_H

class PixelSet
{
protected:
	std::vector<int> _leftBorder;
	std::vector<int> _rightBorder;
	int _ymin;
	int _ymax;

public:
	PixelSet()
	{
		_ymin = 100000;
		_ymax = -1;
		_leftBorder.clear();
		_rightBorder.clear();
	}

	~PixelSet()
	{
		_leftBorder.clear();
		_rightBorder.clear();
	}

	PixelSet& operator= (const PixelSet& rhs)
	{
		_ymin = rhs._ymin;
		_ymax = rhs._ymax;
		_leftBorder.clear();
		_rightBorder.clear();
		_leftBorder = rhs._leftBorder;
		_rightBorder = rhs._rightBorder;

		return *this;
	}

	bool has_pixel() const
	{
		return bottom() <= top() && bottom() > -1 && top() > -1;
	}

	int bottom() const
	{
		return _ymin;
	}

	void set_bottom(const int ymin)
	{
		_ymin = ymin;
	}

	int top() const
	{
		return _ymax;
	}

	void set_top(const int ymax)
	{
		_ymax = ymax;
	}

	int left(const int y) const
	{
		return _leftBorder[y - _ymin];
	}

	int& left(const int y)
	{
		return _leftBorder[y - _ymin];
	}

	int right(const int y) const
	{
		return _rightBorder[y - _ymin];
	}

	int& right(const int y)
	{
		return _rightBorder[y - _ymin];
	}

	std::vector<int>& left()
	{
		return _leftBorder;
	}

	std::vector<int>& right()
	{
		return _rightBorder;
	}
};

#endif

