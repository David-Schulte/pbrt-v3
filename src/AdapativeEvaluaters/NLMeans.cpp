#include "NLMeans.h"
namespace pbrt {
	NLMeans::NLMeans(Film* film, int samplebudget){
		m_film = film;
		m_film_Buffer_A = film->Film_Copy(*film);
		m_film_Buffer_B = film->Film_Copy(m_film_Buffer_A);
		int x_resolution = film->fullResolution.x;
		int y_resolution = film->fullResolution.y;
		std::vector < unsigned int > temp_x_res(x_resolution, samplebudget);
		std::vector < std::vector < unsigned int > > temp_sampleMap(y_resolution, temp_x_res);
		m_sampleMap = std::move(temp_sampleMap);

		std::vector < unsigned int > temp_done_x_res(x_resolution, 0);
		std::vector < std::vector < unsigned int > > temp_doneSampleMap(y_resolution, temp_done_x_res);
		m_doneSampleMap = std::move(temp_doneSampleMap);
		return;
	};

	NLMeans::~NLMeans() {};

	bool NLMeans::hasNextIteration() { return false; };

	void NLMeans::updateSampleMap(){};

	unsigned int NLMeans::getDoneSampleCount(Point2i p0)
	{			
		return m_doneSampleMap.at(p0.y).at(p0.x);
	};

	unsigned int NLMeans::getSampleCount(Point2i p0)
	{
		return m_sampleMap.at(p0.y).at(p0.x);
	};

	NLMeans *CreateNLMeans(Film* film, int samplebudget) 
	{
		return new NLMeans(film, samplebudget);
	}

}