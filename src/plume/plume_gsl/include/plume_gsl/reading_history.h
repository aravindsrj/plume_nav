#include <deque>

#include "geometry_msgs/Point.h"

#pragma once

template<typename T>
class ReadingHistory
{
	std::deque<T> m_array;

	/// \brief Max size of the array
	int m_size;

	/// \brief Records the sum of the elements in the array
	double m_sum;

public:
	
	ReadingHistory();
	void setSize(const int& size);
	void append(const std::pair<double, geometry_msgs::Point>);
	void append(const double);
	void pop();

	/// \brief Return last appended value
	double front() const;

	/// \brief Find mean of the concentrations in the vector of pairs
	double mean() const;

	/// \brief Returns the size of the array
	int size() const;

	/// \brief Returns max size of array
	int maxSize() const;

	/// \brief Find mean value between two positions in a double array
	double mean(const int& begin, const int& end) const;

	/// \brief Clears the array
	void clear();

	geometry_msgs::Point getPoint() const;
};