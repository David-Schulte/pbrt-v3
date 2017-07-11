#ifndef PBRT_ADAPTIVE_EVALUATERS_ADAPTIVE_EVALUATER_H
#define PBRT_ADAPTIVE_EVALUATERS_ADAPTIVE_EVALUATER_H

#include <geometry.h>
#include <vector>
#include <film.h>
namespace pbrt {

	class Adaptive_Evaluater
	{
	public:
		virtual ~Adaptive_Evaluater();
		virtual bool hasNextIteration() = 0 ;
		virtual unsigned int getDoneSampleCount(Point2i p0) = 0;
		virtual unsigned int getSampleCount(Point2i p0) = 0;
		virtual void updateSampleMap() = 0;
		virtual void initialize() = 0;
	};
}
#endif