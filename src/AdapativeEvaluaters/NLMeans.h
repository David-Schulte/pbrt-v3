#ifndef PBRT_ADAPTIVE_EVALUATERS_NL_MEANS_H
#define PBRT_ADAPTIVE_EVALUATERS_NL_MEANS_H

#include "A_Eval.h"

namespace pbrt {
	
	class NLMeans : public Adaptive_Evaluater
	{
	public:
		NLMeans(Film* film, int samplesPerPixel);
		~NLMeans();
		bool hasNextIteration();
		unsigned int getDoneSampleCount(Point2i p0);
		unsigned int getSampleCount(Point2i p0);
		void updateSampleMap();
		Film* getFilm() { return m_film; };

	private:
		Film* m_film;
		Film m_film_Buffer_A;
		Film m_film_Buffer_B;
		bool m_hasNextIteration;
		int m_samplesPerPixel;
		std::vector< std::vector < unsigned int > > m_doneSampleMap;
		std::vector< std::vector < unsigned int > > m_sampleMap;
	};

	NLMeans *CreateNLMeans(Film* film, int samplesPerPixel);
}

#endif