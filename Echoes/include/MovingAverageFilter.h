#ifndef MOVINGAVERAGEFILTER_H
#define MOVINGAVERAGEFILTER_H

class MovingAverageFilter {
	private:
		float alpha;
		float x_k;
	public:
		MovingAverageFilter(float alpha = 0.3) {
			this->alpha = alpha;
		}
		inline float update(float x_knew) {
			x_k = alpha*x_knew + (1-alpha)*x_k;
			return x_k;
		}
		inline float curValue() {
			return x_k;
		}
}; 

#endif
