#include <cstdlib>
#include <iostream>
#include <ctime>
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
		m_sampleMap = std::vector < std::vector < unsigned int > >(m_xResolution, std::vector < unsigned int >(m_yResolution, 0));
		m_doneSampleMap = std::vector < std::vector < unsigned int > >(m_xResolution, std::vector < unsigned int >(m_yResolution, 0));
		m_errorMapA = std::vector < std::vector < Float > >(m_xResolution, std::vector < Float >(m_yResolution, 0));
		m_errorMapB = std::vector < std::vector < Float > >(m_xResolution, std::vector < Float >(m_yResolution, 0));
		m_active_buffer = 0;

		m_r = r;
		m_nghbSize = (2 * r + 1) * (2 * r + 1);
		m_f = f;
		m_patchSize = (2 * f + 1) * (2 * f + 1);
		m_k = k;
		intialize();
	
		m_max_iterations = 5;
		m_iteration = 0;
		m_est_var = estimated_variances(m_xResolution * m_yResolution, std::vector < Float >(3,0));
		return;
	};

	NLMeans::~NLMeans() {};

	bool NLMeans::hasNextIteration()
	{ 
		if (m_iteration++ >= m_max_iterations)
		{
			m_film->pixels.swap(m_pixel_Buffer_A);
			return false;
		}
		if (m_active_buffer == 0)
		{
			m_film->pixels.swap(m_pixel_Buffer_B);
			m_active_buffer = 1;
			return true;
		}
		m_film->pixels.swap(m_pixel_Buffer_A);
		return true;
	
	};

	void NLMeans::intialize()	{
		m_film->pixels.swap(m_pixel_Buffer_A);
		//calculate initial sampleBudget with 40%
		unsigned int initialSampling = ceil(m_sampleBudget * 0.4) ;

		for (std::vector<unsigned int >& i : m_sampleMap)
		{
			std::fill(i.begin(), i.end(), initialSampling);
		}
	};
	void NLMeans::updateSampleMap()
	{

		if (m_active_buffer == 0) return; // update nothing if buffer B wasn´t even processed

		//after this, A is in A again und B is in B again. needed cuz we swap all the time
		m_film->pixels.swap(m_pixel_Buffer_A);
		std::string temp = m_film->filename;

		m_film->filename = "A Inhalt" + std::to_string(m_iteration) + temp;
		m_film->pixels.swap(m_pixel_Buffer_A);
		m_film->WriteImage();
		m_film->pixels.swap(m_pixel_Buffer_A);

		m_film->pixels.swap(m_pixel_Buffer_B);
		m_film->filename = "B Inhalt " + std::to_string(m_iteration) + temp;
		m_film->WriteImage();
		m_film->pixels.swap(m_pixel_Buffer_B);
		m_film->filename = temp;
		
		
		
		//calculatiing estimated variance
		bufferXYZtoRGB(m_pixel_Buffer_A.get());
		bufferXYZtoRGB(m_pixel_Buffer_B.get());
		std::cout << std::endl << "calculating variance";
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		estimateVariance(m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get());
		std::cout << std::endl << "finished calculating variance" << std::endl;
		std::chrono::high_resolution_clock::time_point var = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::seconds> (var - start).count();
		std::cout << std::endl << "Variance estimation took : " << duration;
		// filter A with B but store the result in film, cuz we need unfiltered A for filtering B
		
		filter(m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get(), m_film->pixels.get());
		std::chrono::high_resolution_clock::time_point filt_A = std::chrono::high_resolution_clock::now();
		auto duration1 = std::chrono::duration_cast<std::chrono::seconds> (filt_A - var).count();
		std::cout << std::endl << "A filtered" << std::endl;
		std::cout << std::endl << "It took : " << duration1 << std::endl;

		// now filter B
		filter(m_pixel_Buffer_B.get(), m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get());
		std::cout << std::endl << "B filtered" << std::endl;
		// swap A and film 
		m_film->pixels.swap(m_pixel_Buffer_A);

		// now A and B is filtered and in film is unfiltered A

		std::chrono::high_resolution_clock::time_point start2 = std::chrono::high_resolution_clock::now();
		
		estimateError(m_pixel_Buffer_A.get(), m_pixel_Buffer_B.get(),m_errorMapA);
		std::cout << std::endl << "Error estimated for A";
		estimateError(m_pixel_Buffer_B.get(), m_pixel_Buffer_A.get(), m_errorMapB);
		std::cout << std::endl << "Error estimated for B";
		
		// now create a sample map from the errors
		NLMeans::errorToSampleMap();
		std::chrono::high_resolution_clock::time_point error_t = std::chrono::high_resolution_clock::now();
		auto duration2 = std::chrono::duration_cast<std::chrono::seconds> (error_t - start2).count();
		std::cout << std::endl << "Caculating error and creating sample map took : " << duration2;
		bufferRGBtoXYZ(m_pixel_Buffer_A.get());
		bufferRGBtoXYZ(m_pixel_Buffer_B.get());
		bufferRGBtoXYZ(m_film->pixels.get());

		m_film->filename = "A gefiltert Inhalt" + std::to_string(m_iteration) + temp;
		m_film->pixels.swap(m_pixel_Buffer_A);
		m_film->WriteImage();
		m_film->pixels.swap(m_pixel_Buffer_A);

		m_film->pixels.swap(m_pixel_Buffer_B);
		m_film->filename = "B gefiltert Inhalt " + std::to_string(m_iteration) + temp;
		m_film->WriteImage();
		m_film->pixels.swap(m_pixel_Buffer_B);
		m_film->filename = temp;

		m_active_buffer = 0;
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
		return (m_active_buffer == 0) ? ceil((Float)m_sampleMap.at(p0.x).at(p0.y) / 2.f) : floor((Float)m_sampleMap.at(p0.x).at(p0.y) / 2.f);
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
	NLMeans::Neighbourhood NLMeans::createNeighbourhoodOfPoint(pbrt::Point2i p, int r)
	{
		NLMeans::Neighbourhood ngbh(m_nghbSize);
		// + 1 so that we get 2 * r + 1 values
		pbrt::Bounds2i ngbh_bounds(pbrt::Point2i(p.x - r, p.y - r), pbrt::Point2i(p.x + r + 1, p.y + r + 1));
		int i = 0;
		for (Point2i& point : ngbh_bounds)
		{
			point.x = std::min(std::max(0, point.x), (m_xResolution - 1));
			point.y = std::min(std::max(0, point.y), (m_yResolution - 1));

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
		for (Point2i& point : patch_bounds)
		{
			point.x = std::min(std::max(0, point.x), (m_xResolution - 1));
			point.y =  std::min(std::max(0, point.y), (m_yResolution - 1));
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
		Float distannce = nominator / denominator;
		return distannce;
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


	Float NLMeans::weight(pbrt::Point2i p, pbrt::Point2i q, int r, int f, double k, Pixel_Buffer buffer)
	{
		const NLMeans::Patch Patch_P = createPatchOfPoint(p, f);
		const NLMeans::Patch Patch_Q = createPatchOfPoint(q, f);
		Float weight = std::exp(-std::max(0.f, distanceBetweenTwoPatches(Patch_P, Patch_Q, f, buffer)));
		weight = weight * (weight >= 0.05) ;
		return weight;
	};

	Float NLMeans::finalPatchWeight(pbrt::Point2i p, pbrt::Point2i q, int r, int f, double k)
	{
		// no patchwise yet
		Float sum = 0;

		return 0.0f;
	};

	// calculates average distane between pixels in a patch
	Float NLMeans::distanceBetweenTwoPatches(const Patch P, const Patch Q, int f, Pixel_Buffer buffer)
	{
		Float sum = 0;
		for (int i = 0; i < m_patchSize; i++)
		{

			size_t offset_p = P[i].x + P[i].y * m_xResolution;
			size_t offset_q = Q[i].x + Q[i].y * m_xResolution;

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
					Float var_p = buffer[offset_p].SumOfSqrdDiffsToMean[channel] / (buffer[offset_p].sampleCount - 1);
					Float var_q = buffer[offset_q].SumOfSqrdDiffsToMean[channel] / (buffer[offset_q].sampleCount - 1);
					Float est_var_p;
					Float est_var_q;
					// buffer is m_pixel_buffer_A
					if (buffer == m_pixel_Buffer_A.get())
					{
						est_var_p = std::abs(var_p - (m_pixel_Buffer_B.get()[offset_p].SumOfSqrdDiffsToMean[channel] / m_pixel_Buffer_B.get()[offset_p].sampleCount - 1));
						est_var_q = std::abs(var_q - (m_pixel_Buffer_B.get()[offset_q].SumOfSqrdDiffsToMean[channel] / m_pixel_Buffer_B.get()[offset_q].sampleCount - 1));
					}
					else
					{
						est_var_p = std::abs(var_p - (m_pixel_Buffer_A.get()[offset_p].SumOfSqrdDiffsToMean[channel] / m_pixel_Buffer_A.get()[offset_p].sampleCount - 1));
						est_var_q = std::abs(var_q - (m_pixel_Buffer_A.get()[offset_q].SumOfSqrdDiffsToMean[channel] / m_pixel_Buffer_A.get()[offset_q].sampleCount - 1));
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
	
		NLMeans::Neighbourhood ngbh = createNeighbourhoodOfPoint(p, m_r);
		for( Point2i&  point : ngbh)
		{
			size_t offset_point = point.x + point.y * m_xResolution;

			Float weight;
			if (p == point )
			{
				weight = 1;
			}
			else
			{
				weight = NLMeans::weight(p, point, m_r, m_f, m_k, B);
			}
			normalization += weight;
			for (int channel = 0; channel < 3; channel++)
			{
				if (m_calculating_var)
				{
					sum[channel] += ( std::pow( A[offset_point].xyz[channel] - B[offset_point].xyz[channel],2 ) / 2.f )  * weight;
				}
				else
				{
					sum[channel] += A[offset_point].xyz[channel] * weight;
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
				dst[x + y * m_xResolution] = A[x + y * m_xResolution];
				filterPoint(Point2i(x, y), dst[x + y * m_xResolution].xyz, A, B);
			}
		}
	};

	void NLMeans::estimateVariance(Pixel_Buffer A, Pixel_Buffer B)
	{
		std::unique_ptr<Float[]> rgb(new Float[3 * m_xResolution * m_yResolution]);
		int offset = 0;
		

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
				
				rgb[offset * 3 + 0] = dst[0];
				rgb[offset * 3 + 1] = dst[1];
				rgb[offset * 3 + 2] = dst[2];
				offset++;

			}
		}
		m_r = old_m_r;
		m_f = old_m_f;
		m_k = old_m_k;
		m_nghbSize = (2 * m_r + 1) * (2 * m_r + 1);
		m_patchSize = (2 * m_f + 1) * (2 * m_f + 1);
		m_calculating_var = false;
		
		pbrt::WriteImage("Variance " + std::to_string(m_iteration) + m_film->filename, &rgb[0], m_film->croppedPixelBounds, m_film->fullResolution);
		std::cout << "Image written";
	};
	void NLMeans::estimateError(Pixel_Buffer A, Pixel_Buffer B, std::vector< std::vector < Float > >& m_errorMap)
	{

		for (int i = 0; i < m_xResolution * m_yResolution; i++)
		{
			Float error1 = std::pow((A[i].xyz[0] - B[i].xyz[0]), 2) / (std::pow(A[i].xyz[0], 2) + 0.001);
			Float error2 = std::pow((A[i].xyz[1] - B[i].xyz[1]), 2) / (std::pow(A[i].xyz[1], 2) + 0.001);
			Float error3 = std::pow((A[i].xyz[2] - B[i].xyz[2]), 2) / (std::pow(A[i].xyz[2], 2) + 0.001);
			Float average_error = (error1 + error2 + error3) / Float(3);
			Point2i p = Point2i(i % m_xResolution, floor(i / m_xResolution));

			Neighbourhood ngbh = createNeighbourhoodOfPoint(p, m_r);

			Float error_weight = 0;
			size_t sample_sum = 0;
			for (auto &q : ngbh)
			{
				error_weight += weight(p, q, m_r, m_f, m_k, B);

				size_t offset_point = q.x + q.y * m_xResolution;
				sample_sum = B[offset_point].sampleCount;
			}
			m_errorMap[i % m_xResolution][i / m_xResolution] = (error_weight / Float(1 + sample_sum)) * average_error;;
		}	
	}
	void NLMeans::errorToSampleMap() 
	{
		//first add both error maps and normalise
		std::vector < std::vector < Float > > errors(m_xResolution, std::vector < Float >(m_yResolution, 0));
		//for normalizing store sum
		Float error_sum = 0;

		
		for (int x = 0; x < m_xResolution; x++)
		{
			for (int y = 0; y < m_yResolution; y++)
			{
			Float error = (m_errorMapA.at(x).at(y) + m_errorMapB.at(x).at(y)) / 2;
			error_sum += error;
			errors.at(x).at(y) = error;
			}
		}
		std::unique_ptr<Float[]> rgb(new Float[3 * m_xResolution * m_yResolution]);
		int offset = 0;
		for (int y = 0; y < m_yResolution; y++)
		{
			for (int x = 0; x < m_xResolution; x++)
			{
				rgb[offset * 3 + 0] = errors.at(x).at(y);
				rgb[offset * 3 + 1] = errors.at(x).at(y);
				rgb[offset * 3 + 2] = errors.at(x).at(y);
				offset++;
			}
		}
		pbrt::WriteImage("Error Map " + std::to_string(m_iteration) + m_film->filename, &rgb[0], m_film->croppedPixelBounds, m_film->fullResolution);

		int sample_budget_per_iteration = (m_sampleBudget * 0.6) / m_max_iterations;
		Float scaling_factor = (sample_budget_per_iteration * m_yResolution * m_xResolution) / error_sum;

		std::cout << std::endl << "Sample Budget per Iteration " << sample_budget_per_iteration << std::endl;
		for (int x = 0; x < m_xResolution; x++)
		{
			for (int y = 0; y < m_yResolution; y++)
			{
				errors.at(x).at(y) = errors.at(x).at(y) * scaling_factor;
			}
		}
		// errors are now correctly scaled to contain , on average, as many samples as we wanted to draw in this iteration
		// now caluclate  the transistion from Float error to int sampleCount. 
		// on cases like 3.4 its a 40% change that we draw 4 samples instead of 3. if we draw 3, the 0.4 gets taken into consideration for next calculation
		Float decimal_propagation = 0;
		std::srand(std::time(0));
		unsigned int max_sample_count = 0;
		for (int x = 0; x < m_xResolution; x++)
		{
			for (int y = 0; y < m_yResolution; y++)
			{
				float random_float = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				unsigned int sample_count = floor(errors.at(x).at(y));
				float residual = errors.at(x).at(y) - sample_count;
				// if the random number is in the residual range ( eversthing after the decimal point) + the propagation from the rounds before it gets an extra draw
				if (random_float <= residual + decimal_propagation)
				{
					sample_count++;
					decimal_propagation = 0;
				}
				else
				{
					decimal_propagation += residual;
				}
				if (max_sample_count < sample_count) max_sample_count = sample_count;
				// clamp the sample count , so that they don´t drain far too many samples in one round
				if (sample_count > 4 * sample_budget_per_iteration) sample_count = 4 * sample_budget_per_iteration;

				m_sampleMap.at(x).at(y) = sample_count;
			}
		}
		std::cout << std::endl << "Maximum sample count before clamping was: " << max_sample_count << std::endl;
		offset = 0;
		for (int y = 0; y < m_yResolution; y++)
		{
			for (int x = 0; x < m_xResolution; x++)
			{
				rgb[offset * 3 + 0] = m_sampleMap.at(x).at(y) / sample_budget_per_iteration;
				rgb[offset * 3 + 1] = m_sampleMap.at(x).at(y) / sample_budget_per_iteration;
				rgb[offset * 3 + 2] = m_sampleMap.at(x).at(y) / sample_budget_per_iteration;
				offset++;
			}
		}
		pbrt::WriteImage("Sample Count " + std::to_string(m_iteration) + m_film->filename, &rgb[0], m_film->croppedPixelBounds, m_film->fullResolution);

		
	}
}