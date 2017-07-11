#include "NLMeans.h"
namespace pbrt {
	NLMeans::NLMeans(Film* film, int sampleBudget, int r, int f, double k){
		m_film = film;
		m_pixel_Buffer_A = std::unique_ptr<pbrt::Film::Pixel[]>(new pbrt::Film::Pixel[film->croppedPixelBounds.Area()]);
		m_pixel_Buffer_B = std::unique_ptr<pbrt::Film::Pixel[]>(new pbrt::Film::Pixel[film->croppedPixelBounds.Area()]);
		int x_resolution = film->fullResolution.x;
		int y_resolution = film->fullResolution.y;
		m_sampleBudget = sampleBudget;
		m_sampleMap = std::vector < std::vector < unsigned int > >(x_resolution, std::vector<unsigned int >(y_resolution, 0)); 
		m_doneSampleMap = std::vector < std::vector < unsigned int > >(x_resolution, std::vector<unsigned int >(y_resolution, 0));

		m_r = r;
		m_f = f;
		m_k = k;
		return;
	};

	NLMeans::~NLMeans() {};

	bool NLMeans::hasNextIteration() { return false; };

	void NLMeans::initialize() {};
	void NLMeans::updateSampleMap()
	{
		m_film->pixels.swap(m_pixel_Buffer_A);
	};

	unsigned int NLMeans::getDoneSampleCount(Point2i p0)
	{			
		return m_doneSampleMap.at(p0.x).at(p0.y);
	};

	unsigned int NLMeans::getSampleCount(Point2i p0)
	{
		return m_sampleMap.at(p0.x).at(p0.y);
	};

	NLMeans *CreateNLMeans(Film* film, int samplebudget, int r, int f, double k) 
	{
		return new NLMeans(film, samplebudget, r, f, k);
	}

}