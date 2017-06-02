#ifndef PBRT_ADAPTIVE_EVALUATERS_NL_MEANS_H
#define PBRT_ADAPTIVE_EVALUATERS_NL_MEANS_H

#include "A_Eval.h"
#include <film.h>

namespace pbrt {
	
	class NLMeans : Adaptive_Evaluater
	{
	public:
		NLMeans(Film* film, int samplesPerPixel);
		~NLMeans();
		bool hasNextIteration();
		void updateSampleMap();
		std::vector<Point2i> getPointsInArea(Point2i p0, Point2i p1);

	private:
		Film *m_film;
		int m_samplesPerPixel;
		std::vector< std::vector < unsigned int > > m_sampleMap;
	};
}

#endif