#include "NLMeans.h"
namespace pbrt {
	NLMeans::NLMeans(Film* film, int sampleBudget, int r, int f, double k)
	{
		m_film = film;
		m_pixel_Buffer_A = std::unique_ptr<pbrt::Film::Pixel[]>(new pbrt::Film::Pixel[film->croppedPixelBounds.Area()]);
		m_pixel_Buffer_B = std::unique_ptr<pbrt::Film::Pixel[]>(new pbrt::Film::Pixel[film->croppedPixelBounds.Area()]);
		m_xResolution = film->croppedPixelBounds.pMax.x;
		m_yResolution = film->croppedPixelBounds.pMax.y;
		m_sampleBudget = sampleBudget;
		m_sampleMap = std::vector < std::vector < unsigned int > >(m_xResolution, std::vector<unsigned int >(m_yResolution, 0));
		m_doneSampleMap = std::vector < std::vector < unsigned int > >(m_xResolution, std::vector<unsigned int >(m_yResolution, 0));

		m_active_buffer = 0;

		m_r = r;
		m_nghbSize = (2 * r + 1) * (2 * r + 1);
		m_f = f;
		m_patchSize = (2 * f + 1) * (2 * f + 1);
		m_k = k;
		intialize();
	
		NLMeans::Neighbourhood test = NLMeans::createNeighbourhoodOfPoint(pbrt::Point2i(5, 5), m_film->pixels.get(), m_r);
		NLMeans::Patch patch_test = NLMeans::createPatchInNeighbourhoodOfPoint(pbrt::Point2i(1, 1), test, m_r, m_f);
		return;
	};

	NLMeans::~NLMeans() {};

	bool NLMeans::hasNextIteration()
	{ 
		if (m_active_buffer == 0)
		{
			m_film->pixels.swap(m_pixel_Buffer_B);
			m_active_buffer = 1;
			return true;
		}
		m_film->pixels.swap(m_pixel_Buffer_A);
		return false;
	};

	void NLMeans::intialize()	{
		m_film->pixels.swap(m_pixel_Buffer_A);
		//calculate initial sampleBudget with 20%
		unsigned int initialSampling = m_sampleBudget * 0.2;

		for (std::vector<unsigned int >& i : m_sampleMap)
		{
			std::fill(i.begin(), i.end(), initialSampling);
		}
	};
	void NLMeans::updateSampleMap()
	{

		if (m_active_buffer == 0) return; // update nothing if buffer B wasn´t even processed

		//after this, A is in A again und B is in B again. needed cuz we swap all the time
		m_film->pixels.swap(m_pixel_Buffer_B);
		m_film->pixels.swap(m_pixel_Buffer_A);

		// filter A with B but store the result in film, cuz we need unfiltered A for filtering B
		filter(m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get(), m_film->pixels.get());
		std::cout << "A filtered" << std::endl;
		// now filter B
		filter(m_pixel_Buffer_B.get(), m_film->pixels.get(), m_pixel_Buffer_B.get());
		std::cout << "B filtered" << std::endl;
		// swap A and film 
		m_film->pixels.swap(m_pixel_Buffer_A);
	//	
	};

	unsigned int NLMeans::getDoneSampleCount(Point2i p0)
	{			
		return m_doneSampleMap.at(p0.x).at(p0.y);
	};

	void NLMeans::setDoneSampleCount(Point2i p0, unsigned int count)
	{
		m_doneSampleMap.at(p0.x).at(p0.y) = count ;
	};
	unsigned int NLMeans::getSampleCount(Point2i p0)
	{
		return (m_active_buffer == 0) ? ceil((Float)m_sampleMap.at(p0.x).at(p0.y) / 2.f) : floor((Float) m_sampleMap.at(p0.x).at(p0.y) / 2.f);
	};

	NLMeans *CreateNLMeans(Film* film, int samplebudget, int r, int f, double k) 
	{
		return new NLMeans(film, samplebudget, r, f, k);
	};

	void NLMeans::bufferXYZtoRGB(Pixel_Buffer buff)
	{
		
		//since we work on RGBSpectrum so far, this should be correct and sufficent for now
		for (int i = 0; i < m_xResolution * m_yResolution;i++)
		{
			buff[i].xyz[0] = buff[i].xyz[0] / buff[i].filterWeightSum;
			buff[i].xyz[1] = buff[i].xyz[1] / buff[i].filterWeightSum;
			buff[i].xyz[2] = buff[i].xyz[2] / buff[i].filterWeightSum;
		}
	};
	

	void NLMeans::bufferRGBtoXYZ(Pixel_Buffer buff)
	{
		//since we work on RGBSpectrum so far, this should be correct and sufficent for now
		for (int i = 0; i < m_xResolution * m_yResolution; i++)
		{
			buff[i].xyz[0] = buff[i].xyz[0] * buff[i].filterWeightSum;
			buff[i].xyz[1] = buff[i].xyz[1] * buff[i].filterWeightSum;
			buff[i].xyz[2] = buff[i].xyz[2] * buff[i].filterWeightSum;
		}
	};
	
	// point is from buffer_a, neighbourhood from buffer_b
	NLMeans::Neighbourhood NLMeans::createNeighbourhoodOfPoint(pbrt::Point2i p, Pixel_Buffer buffer, int r)
	{
		NLMeans::Neighbourhood ngbh(m_nghbSize);
		// + 1 so that we get 2 * r + 1 values
		pbrt::Bounds2i ngbh_bounds(pbrt::Point2i(p.x - r, p.y - r), pbrt::Point2i(p.x + r + 1, p.y + r + 1));
		int i = 0;
		for (Point2i point : ngbh_bounds)
		{
			ngbh[i++] = std::move(point);
		}
		return ngbh;
		
	};
	// r is size of neighbourhood and f is size of the patch
	NLMeans::Patch NLMeans::createPatchInNeighbourhoodOfPoint(pbrt::Point2i p, Neighbourhood neighbourhood, int r, int f)
	{
		NLMeans::Patch patch(m_patchSize);
		// + 1 so that we get 2 * f + 1 values
		pbrt::Bounds2i patch_bounds(pbrt::Point2i(p.x - f, p.y - f), pbrt::Point2i(p.x + f + 1, p.y + f + 1));
		int i = 0;
		for (Point2i point : patch_bounds)
		{
			int x_offset_p = std::min(std::max(0,point.x), 2 * r);
			int y_offset_p = std::min(std::max(0,point.y), 2 * r);
			int offset_p = x_offset_p + y_offset_p * (2 * r + 1);

			patch[i++] = neighbourhood[offset_p];
		}
		return patch;
		
	};

	// Pixel should be in RGB and divided by filter sum here, specifiy distance in which color channel with channel parameter
	Float NLMeans::distanceBetweenTwoPixel(pbrt::Film::Pixel& p, pbrt::Film::Pixel& q, int channel)
	{
		// Variance cancellation
		Float alpha = 4;
		Float Var_p = p.SumOfSqrdDiffsToMean[channel] / (p.sampleCount - 1.f);
		Float Var_q = q.SumOfSqrdDiffsToMean[channel] / (q.sampleCount - 1.f);
		Float Var_min = std::min(Var_p, Var_q);

		Float nominator = (p.xyz[channel] - q.xyz[channel]) * (p.xyz[channel] - q.xyz[channel]) - alpha * (Var_p + Var_min);
		Float denominator = 1.0e-10 + m_k * m_k * (Var_p + Var_q);

		return nominator / denominator;
	};

	Float NLMeans::weight(pbrt::Point2i p, pbrt::Point2i q, NLMeans::Neighbourhood ngbh, int r, int f, double k, Pixel_Buffer buffer)
	{
		NLMeans::Patch Patch_P = createPatchInNeighbourhoodOfPoint(p, ngbh, r, f);
		NLMeans::Patch Patch_Q = createPatchInNeighbourhoodOfPoint(q, ngbh, r, f);

		return std::exp(-std::max(0.f, distanceBetweenTwoPatches(Patch_P, Patch_Q, f, buffer)));;
	};

	Float NLMeans::finalPatchWeight(pbrt::Point2i p, pbrt::Point2i q, int r, int f, double k)
	{
		// no patchwise yet
		Float sum = 0;

		return 0.0f;
	};

	// calculates average distane between pixels in a patch
	Float NLMeans::distanceBetweenTwoPatches(Patch P, Patch Q, int f, Pixel_Buffer buffer)
	{
		Float sum = 0;
		for (int channel = 0; channel < 3; channel++)
		{
			for (int i = 0; i < m_patchSize; i++)
			{
				int x_offset_p = std::min(std::max(0, P[i].x), 2 * f);
				int y_offset_p = std::min(std::max(0, P[i].y), 2 * f);
				int offset_p = x_offset_p + y_offset_p * (2 * f + 1);

				int x_offset_q = std::min(std::max(0, Q[i].x), 2 * f);
				int y_offset_q = std::min(std::max(0, Q[i].y), 2 * f);
				int offset_q = x_offset_q + y_offset_q * (2 * f + 1);

				// the distanceBetweenTwoPixels is 0 if they are the same
				if (offset_p == offset_q)
				{
					sum += 0;
					continue;
				}
				sum += distanceBetweenTwoPixel(buffer[offset_p], buffer[offset_q], channel);
			}
		}
		return sum / (3.f * ( (Float) m_patchSize ));
	}; 

	void NLMeans::filterPoint(pbrt::Point2i p, Float dst[3], Pixel_Buffer A, Pixel_Buffer B)
	{
		Float sum[3];
		sum[0] = sum[1] = sum[2] = 0.f;
		Float normalization = 0;

		dst[0] = A[p.x + p.y * m_xResolution].xyz[0];
		dst[1] = A[p.x + p.y * m_xResolution].xyz[1];
		dst[2] = A[p.x + p.y * m_xResolution].xyz[2];
		NLMeans::Neighbourhood ngbh = createNeighbourhoodOfPoint(p, B, m_r);
		for (int y = p.y - m_r; y < p.y + m_r; y++)
		{
			if (y < 0 || y >= m_yResolution) continue;
			for (int x = p.x - m_r; x < p.x + m_r; x++)
			{
				if (x < 0 || x >= m_xResolution) continue;
				Float weight;
				if (p == Point2i(x, y))
				{
					weight = 1;
				}
				else
				{
					weight = NLMeans::weight(p, Point2i(x, y), ngbh, m_r, m_f, m_k, B);
				}
				normalization += weight;
				for (int channel = 0; channel < 3; channel++)
				{
					sum[channel] += dst[channel] * weight;
				}
			}
		}
		dst[0] = sum[0] / normalization;
		dst[1] = sum[1] / normalization;
		dst[2] = sum[2] / normalization;

	};
	void NLMeans::filter(Pixel_Buffer A, Pixel_Buffer B, Pixel_Buffer dst)
	{
		for (int y = 0; y < m_yResolution; y++)
		{
			for (int x = 0; x < m_xResolution; x++)
			{
				if (x == 1135 && y == 7)
				{
					std::cout << "Stop" << std::endl;
				}
				std::cout << "Point: " << Point2i(x, y) << "before filtering: " << A[x + y * m_xResolution].xyz[0] << std::endl;
				filterPoint(Point2i(x, y), dst[x + y * m_xResolution].xyz, A, B);
				std::cout << "After filtering: " << dst[x + y * m_xResolution].xyz[0] << std::endl;
			}
		}
	};
}