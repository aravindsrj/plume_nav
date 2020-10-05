#include "plume_gsl/reading_history.h"

template<>
ReadingHistory<std::pair<double, geometry_msgs::Point>>::ReadingHistory():
m_sum(0.0),
m_size(-1)
{
}

template<>
ReadingHistory<double>::ReadingHistory():
m_sum(0.0),
m_size(-1)
{
}

template<>
void ReadingHistory<std::pair<double, geometry_msgs::Point>>::setSize(const int& size)
{
	m_size = size;
}

template<>
void ReadingHistory<double>::setSize(const int& size)
{
	m_size = size;
}

template<>
void ReadingHistory<std::pair<double, geometry_msgs::Point>>::append(
	const std::pair<double, geometry_msgs::Point> data)
{
	m_sum += data.first;
	array.push_back(data);
	if (array.size() > m_size)
	{
		m_sum -= array[0].first;
		array.pop_front();
	}
	assert(m_sum >= 0);
}

template<>
void ReadingHistory<double>::append(const double data)
{
	array.push_back(data);
	if (m_size != -1) // If the max_size variable is set
	{
		if (array.size() > m_size)
		{
			array.pop_front();
		}
	}
}

template<typename T>
void ReadingHistory<T>::pop()
{
	array.pop_front();
}

template<>
double ReadingHistory<std::pair<double, geometry_msgs::Point>>::mean() const
{
	return m_sum/array.size();
}

template<>
int ReadingHistory<std::pair<double,geometry_msgs::Point>>::size() const
{
	return array.size();
}

template<>
int ReadingHistory<double>::size() const
{
	return array.size();
}

template<>
double ReadingHistory<double>::mean(const int& begin, const int& end) const
{
	assert(0 <= begin <= end);
	assert(end <= array.size());

	if (begin == end)
		return array[begin];

	double sum = 0;
	for (int i = begin; i < end; i++)
	{
		sum += array[i];
	}

	return sum/(end-begin);
}

template<>
geometry_msgs::Point ReadingHistory<std::pair<double,geometry_msgs::Point>>::getPoint() const
{
	assert(array.size() > 0);
	return array[array.size()/2].second;
}

template<>
void ReadingHistory<double>::clear()
{
	array.clear();
	m_sum = 0.0;
}

template<>
void ReadingHistory<std::pair<double,geometry_msgs::Point>>::clear()
{
	array.clear();
	m_sum = 0.0;
}