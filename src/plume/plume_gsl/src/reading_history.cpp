#include "plume_gsl/reading_history.h"

template<typename T>
ReadingHistory<T>::ReadingHistory():
m_sum(0.0),
m_size(-1)
{
}

// Explicit instantiation
template class ReadingHistory<double>;
template class ReadingHistory<std::pair<double, geometry_msgs::Point>>;

template<typename T>
void ReadingHistory<T>::setSize(const int& size)
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
	m_sum += data;
	array.push_back(data);
	if (m_size != -1) // If the max_size variable is set
	{
		if (array.size() > m_size)
		{
			m_sum -= array[0];
			array.pop_front();
		}
	}
	assert(m_sum >= 0);
}

template<typename T>
void ReadingHistory<T>::pop()
{
	array.pop_front();
}

template<typename T>
double ReadingHistory<T>::mean() const
{
	if (array.size() == 0)
		throw "Array is empty. Cannot find mean";
	return m_sum/array.size();
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

template<typename T>
int ReadingHistory<T>::size() const
{
	return array.size();
}

template<typename T>
int ReadingHistory<T>::maxSize() const
{
	return m_size;
}

template<>
geometry_msgs::Point ReadingHistory<std::pair<double,geometry_msgs::Point>>::getPoint() const
{
	assert(array.size() > 0);

	// Returns the the point at the median of the array
	return array[array.size()/2].second;
}

template<typename T>
void ReadingHistory<T>::clear()
{
	array.clear();
	m_sum = 0.0;
}