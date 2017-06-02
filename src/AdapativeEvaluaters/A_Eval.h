#ifndef PBRT_ADAPTIVE_EVALUATERS_ADAPTIVE_EVALUATER_H
#define PBRT_ADAPTIVE_EVALUATERS_ADAPTIVE_EVALUATER_H

#include <geometry.h>
#include <vector>

namespace pbrt {

	class Adaptive_Evaluater
	{
	public:
		virtual ~Adaptive_Evaluater();
		virtual bool hasNextIteration() = 0 ;
		virtual std::vector<Point2i> getPointsInArea(Point2i p0, Point2i p1) = 0;
		virtual void updateSampleMap() = 0;

	};
}
#endif