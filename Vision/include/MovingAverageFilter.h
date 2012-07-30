#ifndef MOVING_AVERAGE_FILTER_H
#define MOVING_AVERAGE_FILTER_H

class MovingAverageFilter
{
	private:
		float x_k;
		float alpha;
	public:
		MovingAverageFilter() : x_k(0), alpha(0.125) {};
		inline float update(float x_knew)
		{
			x_k = alpha * x_knew + (1 - alpha) * x_k;
			return x_k;
		}
		inline float curValue()
		{
			return x_k;
		}
};

#endif
