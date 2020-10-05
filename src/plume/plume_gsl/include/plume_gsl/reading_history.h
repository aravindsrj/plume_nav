#include <deque>

#include "geometry_msgs/Point.h"

#pragma once

template<typename T>
class ReadingHistory
{
	std::deque<T> array;
	int m_size;
	double m_sum;

public:
	
	ReadingHistory();
	void setSize(const int& size);
	void append(const std::pair<double, geometry_msgs::Point>);
	void append(const double);
	void pop();

	/// \brief Find mean of the concentrations in the vector of pairs
	double mean() const;

	int size() const;

	/// \brief Find mean value between two positions in a double array
	double mean(const int& begin, const int& end) const;
	void clear();
	geometry_msgs::Point getPoint() const;
};