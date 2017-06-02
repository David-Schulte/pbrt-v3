#include "NLMeans.h"
namespace pbrt {
	NLMeans::NLMeans(Film* film,int samplesPerPixel){
		m_film = film;
		int x_resolution = film->fullResolution.x;
		int y_resolution = film->fullResolution.y;
		std::vector < unsigned int > temp_x_res(x_resolution, 1);
		std::vector < std::vector < unsigned int > > temp_sampleMap(y_resolution, temp_x_res);
		m_sampleMap = temp_sampleMap;
		return;
	};

	NLMeans::~NLMeans() {};

	bool NLMeans::hasNextIteration() { return false; };

	void NLMeans::updateSampleMap(){};

	std::vector<Point2i> NLMeans::getPointsInArea(Point2i p0, Point2i p1){
		std::vector<Point2i> pointsInArea;
		
		for (int i = p0.y; i < p1.y; i++) // loop in y direction
		{
			for (int j = p0.x; j < p1.x; j++) //copy all values in x direction
			{
					pointsInArea.push_back(Point2i(j, i));
			}
		}
		return pointsInArea;
	};
}