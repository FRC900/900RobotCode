#ifndef _OPT_PARAMS_LIST_
#define _OPT_PARAMS_LIST_

#include <vector>
#include "base_trajectory/opt_params.h"

template <class T>
class OptParamsList
{
	public:
		// Code to mimic a vector
		OptParams<T> &operator[] (size_t index);
		const OptParams<T> &operator[] (size_t index) const;
		OptParams<T> &back(void);
		const OptParams<T> &back(void) const;
		size_t size(void) const;
		void push_back(const OptParams<T> &params);
		typename std::vector<OptParams<T>>::iterator begin();
		typename std::vector<OptParams<T>>::iterator end();
		typename std::vector<OptParams<T>>::const_iterator begin() const;
		typename std::vector<OptParams<T>>::const_iterator end() const;
		typename std::vector<OptParams<T>>::const_iterator cbegin() const;
		typename std::vector<OptParams<T>>::const_iterator cend() const;
		// TODO - addPoint()
		//
		void toVector(std::vector<T> &v) const;
		void fromVector(const std::vector<T> &v);
		void getLimits(std::vector<T> &lower,std::vector<T> &upper) const;

	private:
		std::vector<OptParams<T>> optParams_;

};

#endif
