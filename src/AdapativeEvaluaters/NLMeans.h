#ifndef PBRT_ADAPTIVE_EVALUATERS_NL_MEANS_H
#define PBRT_ADAPTIVE_EVALUATERS_NL_MEANS_H

#include "A_Eval.h"

namespace pbrt {
	
	class NLMeans : public Adaptive_Evaluater
	{
	public:
		NLMeans(Film* film, int sampleBudget, int r, int f, double k);
		~NLMeans();
		bool hasNextIteration();
		unsigned int getDoneSampleCount(Point2i p0);
		unsigned int getSampleCount(Point2i p0);
		void updateSampleMap() _NOEXCEPT;
		void initialize();
	private:
		// Reference to the film the camera holds and 
		Film* m_film;
		// Buffer A in NLMeans Paper 
		std::unique_ptr<pbrt::Film::Pixel[]> m_pixel_Buffer_A;
		// Buffer B in NLMeans Paper
		std::unique_ptr<pbrt::Film::Pixel[]> m_pixel_Buffer_B;
		// is true if the nl means filter thinks it should have anothier iteration
		bool m_hasNextIteration;
		// the user given sample budget ergo the orginial desired sample number per pixel
		int m_sampleBudget;
		// m_r describes the size of the square neighborhood   used size in calculations is 2r + 1
		int m_r;
		// m_f describes the size of the square patches   used size in caculations is 2f + 1
		int m_f;
		// m_k is the user specified damping factor which controls the strengt of the filter. Lower value results in a more conservative filter
		double m_k;
		// epsilon  used during distance computation so prevent division by zero
		const double e = 1e-10;
		// map which stores how often the pixel will be sampled
		std::vector< std::vector < unsigned int > > m_sampleMap;
		// map which stores how often a pixel got sampled
		std::vector< std::vector < unsigned int > > m_doneSampleMap;
	};

	NLMeans *CreateNLMeans(Film* film, int samplesPerPixel, int r, int f, double k);
}

#endif