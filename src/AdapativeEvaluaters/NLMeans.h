#ifndef PBRT_ADAPTIVE_EVALUATERS_NL_MEANS_H
#define PBRT_ADAPTIVE_EVALUATERS_NL_MEANS_H

#include "A_Eval.h"
#include "imageio.h"
namespace pbrt {
	
	class NLMeans : public Adaptive_Evaluater
	{
	// flat vector for pixel in a Patch
	typedef std::vector < pbrt::Point2i > Patch;
		
	// flat vector for pixel
	typedef std::vector < pbrt::Point2i > Neighbourhood;
		
	// unique ptr on flat array of Pixel
	typedef pbrt::Film::Pixel* __restrict Pixel_Buffer;

	//vector of estimated variances
	typedef std::vector < std::vector< Float > > estimated_variances;

	public:
		NLMeans(Film* film, int sampleBudget, int r, int f, double k);
		~NLMeans();
		bool hasNextIteration();
		unsigned int getDoneSampleCount(Point2i p0);
		void setDoneSampleCount(Point2i p0, unsigned int count);
		unsigned int getSampleCount(Point2i p0);
		void updateSampleMap();
		
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
		// m_nghbSize is the nr of elements in a neighborhood
		int m_nghbSize;
		// m_f describes the size of the square patches   used size in caculations is 2f + 1
		int m_f;
		// nr of elements in a patch
		int m_patchSize;
		// m_k is the user specified damping factor which controls the strengt of the filter. Lower value results in a more conservative filter
		double m_k;
		// epsilon  used during distance computation so prevent division by zero
		const double e = 1e-10;
		// map which stores how often the pixel will be sampled
		std::vector< std::vector < unsigned int > > m_sampleMap;
		// map which stores how often a pixel got sampled
		std::vector< std::vector < unsigned int > > m_doneSampleMap;
		// map which stores error of buffer A;
		std::vector< std::vector < Float > > m_errorMapA;
		// map which stores error of buffer B;
		std::vector< std::vector < Float > > m_errorMapB;

		int m_xResolution = 0;
		int m_yResolution = 0;
		int m_active_buffer = 0;
		bool m_calculating_var = 0;
		int m_max_iterations = 0;
		int m_iteration = 0;
		estimated_variances m_est_var;

		void intialize();

		void bufferXYZtoRGB(Pixel_Buffer buff);

		void bufferRGBtoXYZ(Pixel_Buffer buff);
		
		// neighbourhood size is r * r + 1
		Neighbourhood createNeighbourhoodOfPoint(pbrt::Point2i p, int r);
		
		Patch NLMeans::createPatchOfPoint(pbrt::Point2i p, int f);
	
		// Pixel should be in RGB and divided by filter sum here, specifiy distance in which color channel with channel parameter
		Float distanceBetweenTwoPixel(pbrt::Film::Pixel& p, pbrt::Film::Pixel& q, int channel, int offset_p, int offset_q);

		// Pixel should be in RGB and divided by filter sum here, specifiy distance in which color channel with channel parameter
		Float distanceBetweenTwoVariances(Float var_p, Float var_q, Float est_var_p, Float est_var_q);
		
		//estimate variance
		void estimateVariance(Pixel_Buffer A, Pixel_Buffer B);
		
		Float weight(pbrt::Point2i p, pbrt::Point2i q, int r, int f, double k, Pixel_Buffer buffer);

		Float finalPatchWeight(pbrt::Point2i p, pbrt::Point2i q, int r, int f, double k);
		
		// caalculates average distane between pixels in a patch
		Float distanceBetweenTwoPatches(const Patch P, const Patch Q, int f, Pixel_Buffer buffer);

		// filter a single Point
		void filterPoint(pbrt::Point2i p, Float u[3], Pixel_Buffer A, Pixel_Buffer B);

		// filter complete buffer A with weights from buffer B
		void filter(Pixel_Buffer A, Pixel_Buffer B, Pixel_Buffer C);

		//estimate error
		void NLMeans::estimateError(Pixel_Buffer A, Pixel_Buffer B, std::vector< std::vector < Float > >& m_errorMap);

		// create sampleMap from error values
		void errorToSampleMap();
	};

	NLMeans *CreateNLMeans(Film* film, int samplesPerPixel, int r, int f, double k);
}

#endif