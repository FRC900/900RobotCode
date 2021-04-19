#include "base_trajectory/opt_params_list.h"

// Code to mimic a vector
template<class T>
OptParams<T> &OptParamsList<T>::operator[] (size_t index)
{
	return optParams_[index];
}

template<class T>
const OptParams<T> &OptParamsList<T>::operator[] (size_t index) const
{
	return optParams_[index];
}

template<class T>
OptParams<T> &OptParamsList<T>::back(void)
{
	return optParams_.back();
}

template<class T>
const OptParams<T> &OptParamsList<T>::back(void) const
{
	return optParams_.back();
}

template<class T>
size_t OptParamsList<T>::size(void) const
{
	return optParams_.size();
}

template<class T>
void OptParamsList<T>::push_back(const OptParams<T> &params)
{
	optParams_.push_back(params);
}

template<class T>
typename std::vector<OptParams<T>>::iterator OptParamsList<T>::begin()
{
	return optParams_.begin();
}

template<class T>
typename std::vector<OptParams<T>>::iterator OptParamsList<T>::end()
{
	return optParams_.end();
}

template<class T>
typename std::vector<OptParams<T>>::const_iterator OptParamsList<T>::cbegin() const
{
	return optParams_.cbegin();
}

template<class T>
typename std::vector<OptParams<T>>::const_iterator OptParamsList<T>::cend() const
{
	return optParams_.cend();
}

template<class T>
typename std::vector<OptParams<T>>::const_iterator OptParamsList<T>::begin() const
{
	return optParams_.cbegin();
}

template<class T>
typename std::vector<OptParams<T>>::const_iterator OptParamsList<T>::end() const
{
	return optParams_.cend();
}
// TODO - addPoint()

template<class T>
void OptParamsList<T>::toVector(std::vector<T> &v) const
{
	v.clear();
	for (const auto &op : optParams_)
	{
		for (size_t i = 0; i < op.size(); i++)
		{
			if (!op.doNotOptimize(i))
			{
				v.push_back(op[i]);
			}
		}
	}
}
template<class T>
void OptParamsList<T>::fromVector(const std::vector<T> &v)
{
	size_t j = 0;
	for (auto &op : optParams_)
	{
		for (size_t i = 0; i < op.size(); i++)
		{
			if (!op.doNotOptimize(i))
			{
				op[i] = v[j++];
			}
		}
	}
}
template<class T>
void OptParamsList<T>::getLimits(std::vector<T> &lower,std::vector<T> &upper) const
{
	lower.clear();
	upper.clear();
	for (const auto &op : optParams_)
	{
		for (size_t i = 0; i < op.size(); i++)
		{
			if (!op.doNotOptimize(i))
			{
				lower.push_back(op.getLowerLimit(i));
				upper.push_back(op.getUpperLimit(i));
			}
		}
	}
}


template class OptParamsList<double>;
