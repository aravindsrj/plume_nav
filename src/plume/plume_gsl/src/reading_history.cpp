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
	m_array.push_back(data);
	if (m_array.size() > m_size)
	{
		m_sum -= m_array[0].first;
		m_array.pop_front();
	}
	assert(m_sum >= 0);
}

template<>
void ReadingHistory<double>::append(const double data)
{
	m_array.push_back(data);
	if (m_size != -1) // If the max_size variable is set
	{
		m_sum += data;
		if (m_array.size() > m_size)
		{
			m_sum -= m_array[0];
			m_array.pop_front();
		}
		
	}
	assert(m_sum >= 0);
}

template<typename T>
void ReadingHistory<T>::pop()
{
	m_array.pop_front();
}

template<>
double ReadingHistory<double>::back() const
{
	return m_array.back();
}

template<typename T>
double ReadingHistory<T>::mean() const
{
	// The following assertion is required because m_sum will be calculated only
	// if m_size is not equal to -1
	assert(m_size >= 0);

	if (m_array.size() == 0)
		throw "m_array is empty. Cannot find mean";
	return m_sum/m_array.size();
}


template<>
double ReadingHistory<double>::mean(const int& begin, const int& end) const
{
	assert(0 <= begin <= end);
	assert(end <= m_array.size());

	if (begin == end)
		return m_array[begin];

	double sum = 0;
	for (int i = begin; i < end; i++)
	{
		sum += m_array[i];
	}

	return sum/(end-begin);
}

template<typename T>
int ReadingHistory<T>::size() const
{
	return m_array.size();
}

template<typename T>
int ReadingHistory<T>::maxSize() const
{
	return m_size;
}

template<>
geometry_msgs::Point ReadingHistory<std::pair<double,geometry_msgs::Point>>::getPoint() const
{
	assert(m_array.size() > 0);

	// Returns the the point at the median of the m_array
	return m_array[m_array.size()/2].second;
}

template<typename T>
void ReadingHistory<T>::clear()
{
	m_array.clear();
	m_sum = 0.0;
}