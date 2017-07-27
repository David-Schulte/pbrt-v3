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
		NLMeans::Patch patch_test = NLMeans::createPatchOfPoint(pbrt::Point2i(4, 4), m_f);

		m_est_var = estimated_variances(m_xResolution * m_yResolution, std::vector < Float >(3,0));
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
		unsigned int initialSampling = m_sampleBudget * 0.2 + 1;

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
		//calculatiing estimated variance
	//	bufferXYZtoRGB(m_pixel_Buffer_A.get());
	//	bufferXYZtoRGB(m_pixel_Buffer_B.get());
		std::cout << std::endl << "calculating variance";
		estimateVariance(m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get());
		std::cout << std::endl << "finished calculating variance" << std::endl;
		// filter A with B but store the result in film, cuz we need unfiltered A for filtering B
		filter(m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get(), m_film->pixels.get());
		std::cout << "A filtered" << std::endl;
		// now filter B
		filter(m_pixel_Buffer_B.get(), m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get());
		std::cout << "B filtered" << std::endl;
		// swap A and film 
		m_film->pixels.swap(m_pixel_Buffer_A);

		// now A and B is filtered and in film is unfiltered A

		estimateError(m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get());
		m_film->pixels.swap(m_pixel_Buffer_B);
	//	bufferRGBtoXYZ(m_pixel_Buffer_A.get());
	//	bufferRGBtoXYZ(m_pixel_Buffer_B.get());
	//	bufferRGBtoXYZ(m_film->pixels.get());
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
	NLMeans::Patch NLMeans::createPatchOfPoint(pbrt::Point2i p, int f)
	{
		NLMeans::Patch patch(m_patchSize);
		// + 1 so that we get 2 * f + 1 values
		pbrt::Bounds2i patch_bounds(pbrt::Point2i(p.x - f, p.y - f), pbrt::Point2i(p.x + f + 1, p.y + f + 1));
		int i = 0;
		for (Point2i point : patch_bounds)
		{
			patch[i++] = std::move(point); 
		}
		return patch;
		
	};

	// Pixel should be in RGB and divided by filter sum here, specifiy distance in which color channel with channel parameter
	Float NLMeans::distanceBetweenTwoPixel(pbrt::Film::Pixel& p, pbrt::Film::Pixel& q, int channel, int offset_p, int offset_q)
	{
		// Variance cancellation
		Float alpha = 0.5;
		Float Var_p = m_est_var[offset_p][channel];
		Float Var_q = m_est_var[offset_q][channel];
		Float Var_min = std::min(Var_p, Var_q);

		Float nominator = std::pow((p.xyz[channel] - q.xyz[channel]),2) - alpha * (Var_p + Var_min);
		Float denominator = 1.0e-10 + m_k * m_k * (Var_p + Var_q);

		return nominator / denominator;
	};

	Float NLMeans::distanceBetweenTwoVariances(Float var_p, Float var_q, Float est_var_p, Float est_var_q)
	{
		// Variance cancellation
		Float alpha = 4;
		Float Var_min = std::min(est_var_p, est_var_q);
		Float nominator = (var_p - var_q) * (var_p - var_q) - alpha * (est_var_p + Var_min);
		Float denominator = 1.0e-10 + m_k * m_k * (est_var_p + est_var_q);

		return nominator / denominator;
	};


	Float NLMeans::weight(pbrt::Point2i p, pbrt::Point2i q, NLMeans::Neighbourhood ngbh, int r, int f, double k, Pixel_Buffer buffer)
	{
		NLMeans::Patch Patch_P = createPatchOfPoint(p, f);
		NLMeans::Patch Patch_Q = createPatchOfPoint(q, f);

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
		for (int i = 0; i < m_patchSize; i++)
		{
			int x_offset_p = std::min(std::max(0, P[i].x), (m_xResolution - 1) );
			int y_offset_p = std::min(std::max(0, P[i].y), (m_yResolution - 1) );
			size_t offset_p = x_offset_p + y_offset_p * m_xResolution;

			int x_offset_q = std::min(std::max(0, Q[i].x), (m_xResolution - 1) );
			int y_offset_q = std::min(std::max(0, Q[i].y), (m_yResolution - 1) );
			size_t offset_q = x_offset_q + y_offset_q * m_xResolution;
			// the distanceBetweenTwoPixels is 0 if they are the sames
			if (offset_p == offset_q)
			{
				sum += 0;
				continue;
			}
			if (m_calculating_var)
			{
				for (int channel = 0; channel < 3; channel++)
				{
					Float var_p = buffer[offset_p].SumOfSqrdDiffsToMean[channel] / buffer[offset_p].sampleCount;
					Float var_q = buffer[offset_q].SumOfSqrdDiffsToMean[channel] / buffer[offset_q].sampleCount;
					Float est_var_p;
					Float est_var_q;
					// buffer is m_pixel_buffer_A
					if (buffer == m_pixel_Buffer_A.get())
					{
						est_var_p = std::pow((var_p - m_pixel_Buffer_B.get()[offset_p].SumOfSqrdDiffsToMean[channel] / m_pixel_Buffer_B.get()[offset_p].sampleCount), 2) / 2.f;
						est_var_q = std::pow((var_q - m_pixel_Buffer_B.get()[offset_q].SumOfSqrdDiffsToMean[channel] / m_pixel_Buffer_B.get()[offset_q].sampleCount), 2) / 2.f;
					}
					else
					{
						est_var_p = std::pow((var_p - m_pixel_Buffer_A.get()[offset_p].SumOfSqrdDiffsToMean[channel] / m_pixel_Buffer_A.get()[offset_p].sampleCount), 2) / 2.f;
						est_var_q = std::pow((var_q - m_pixel_Buffer_A.get()[offset_q].SumOfSqrdDiffsToMean[channel] / m_pixel_Buffer_A.get()[offset_q].sampleCount), 2) / 2.f;
					}
					sum += distanceBetweenTwoVariances(var_p, var_q, est_var_p, est_var_q);
				}
			}
			else
			{
				for (int channel = 0; channel < 3; channel++)
				{
					sum += distanceBetweenTwoPixel(buffer[offset_p], buffer[offset_q], channel, offset_p,offset_q);
				}
			}
		}
		return sum / (3.f * ( (Float) m_patchSize ));
	}; 

	void NLMeans::filterPoint(pbrt::Point2i p, Float dst[3], Pixel_Buffer A, Pixel_Buffer B)
	{
		Float sum[3];
		sum[0] = sum[1] = sum[2] = 0.f;
		Float normalization = 0;

		if (m_calculating_var)
		{
			dst[0] = A[p.x + p.y * m_xResolution].SumOfSqrdDiffsToMean[0] / A[p.x + p.y * m_xResolution].sampleCount;
			dst[1] = A[p.x + p.y * m_xResolution].SumOfSqrdDiffsToMean[1] / A[p.x + p.y * m_xResolution].sampleCount;
			dst[2] = A[p.x + p.y * m_xResolution].SumOfSqrdDiffsToMean[2] / A[p.x + p.y * m_xResolution].sampleCount;
		}
		else
		{
			dst[0] = A[p.x + p.y * m_xResolution].xyz[0];
			dst[1] = A[p.x + p.y * m_xResolution].xyz[1];
			dst[2] = A[p.x + p.y * m_xResolution].xyz[2];
		}
		NLMeans::Neighbourhood ngbh = createNeighbourhoodOfPoint(p, B, m_r);
		for( Point2i  point : ngbh)
		{
			Float weight;
			if (p == point )
			{
				weight = 1;
			}
			else
			{
				weight = NLMeans::weight(p, point, ngbh, m_r, m_f, m_k, B);
			}
			normalization += weight;
			for (int channel = 0; channel < 3; channel++)
			{
				sum[channel] += dst[channel] * weight;
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
				Float val_before = A[x + y * m_xResolution].xyz[0];


				filterPoint(Point2i(x, y), dst[x + y * m_xResolution].xyz, A, B);
				Float val_after = dst[x + y * m_xResolution].xyz[0];
				if (std::abs(val_before - val_after) > 0.05) std::cout << "Point difference: " << Point2i(x, y) << "  "  << val_before - val_after << std::endl << std::endl;
			}
		}
	};

	void NLMeans::estimateVariance(Pixel_Buffer A, Pixel_Buffer B)
	{
		m_calculating_var = true;
		int old_m_r = m_r;
		int old_m_f = m_f;
		Float old_m_k = m_k;
		m_r = 1;
		m_f = 3;
		m_k = 0.45;
		m_nghbSize = (2 * m_r + 1) * (2 * m_r + 1);
		m_patchSize = (2 * m_f + 1) * (2 * m_f + 1);

		for (int y = 0; y < m_yResolution; y++)
		{
			for (int x = 0; x < m_xResolution; x++)
			{
				Float dst[3] = { 0, 0, 0 } ;
				filterPoint(Point2i(x, y), dst, A, B);
				
				m_est_var[x + y * m_xResolution][0] = dst[0];
				m_est_var[x + y * m_xResolution][1] = dst[1];
				m_est_var[x + y * m_xResolution][2] = dst[2];
				
			}
		}
		m_r = old_m_r;
		m_f = old_m_f;
		m_k = old_m_k;
		m_nghbSize = (2 * m_r + 1) * (2 * m_r + 1);
		m_patchSize = (2 * m_f + 1) * (2 * m_f + 1);
		m_calculating_var = false;
	};
	void NLMeans::estimateError(Pixel_Buffer A, Pixel_Buffer B)
	{
		for (int i = 0; i < m_xResolution * m_yResolution; i++)
		{
			Float error1 = std::pow((A->xyz[0] - B->xyz[0]), 2) / std::pow(A->xyz[0],2);
			Float error2 = std::pow((A->xyz[1] - B->xyz[1]), 2) / std::pow(A->xyz[1], 2);
			Float error3 = std::pow((A->xyz[2] - B->xyz[2]), 2) / std::pow(A->xyz[2], 2);

			if ((error1 + error2 + error3) > 1) std::cout << "error value: " << error1 + error2 + error3 << std::endl;
		}

	}
}