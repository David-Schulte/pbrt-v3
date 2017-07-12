#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_NLMEANSFILTER_H
#define PBRT_CORE_NLMEANSFILTER_H

#include "sampling.h"

namespace pbrt
{

    class NLMeansFilter
    {
		// flat vector for pixel in a Patch
		typedef std::vector <  pbrt::Pixel  > Patch;  

		// flat vector for pixel
		typedef std::vector <  pbrt::Pixel  > Neighbourhood;

		// unique ptr on flat array of Pixel
		typedef std::unique_ptr < pbrt::Pixel[] > Pixel_Buffer;
    public:
        NLMeansFilter();
        ~NLMeansFilter();

        std::vector<std::vector<std::vector<float>>> Filter(Film * film, int weightSourceBuffer, int filterBuffer, int filterRadius, int patchRadius);

		// static function to filter an array of pbrt::Pixel (buffer_a) with the NLMeans filter, but calculating the weights from buffer_b
		// storing result in buffer dst
		static void filter_buffer_dest_with_src(Pixel_Buffer buffer_a, Pixel_Buffer buffer_b, Pixel_Buffer dst, int r, int f, int k);

    protected:
        std::vector<float> FilterPixel(Film * film, Point2i pixel, int weightSourceBuffer, int filterBuffer, int filterRadius, int patchRadius);
        float PatchWeight(Film * film, int buffer, Point2i pixel1, Point2i pixel2, int radius, float dampingFactor);
        float PatchDistance(Film * film, int buffer, Point2i pixel1, Point2i pixel2, int radius);
        Bounds2i SharedBounds(Film * film, std::vector<Point2i> pixels, int radius);

		// Davids  static functions

		static void bufferXYZtoRGB(Pixel_Buffer buff);

		static void bufferRGBtoXYZ(Pixel_Buffer buff);

		// point is from buffer_a, neighbourhood from buffer_b
		static Neighbourhood createNeighbourhoodOfPoint(pbrt::Point2i, Pixel_Buffer buffer_a, Pixel_Buffer buffer_b, int r);

		static Patch createPatchInNeighbourhoodOfPoint(pbrt::Point2i, Neighbourhood neighbourhood, int r,  int f);

		// Pixel should be in RGB here, specifiy distance in which color channel with channel parameter
		static Float distanceBetweenTwoPixel(pbrt::Pixel p, pbrt::Pixel q, int channel);

		static Float weight(pbrt::Pixel p, pbrt::Pixel q, int r, int f, double k);
		static float finalPatchWeight(pbrt::Pixel p, pbrt::Pixel q, int r, int f, double k);

		// caöciöates average distane between pixels in a patch
		static Float distanceBetweenTwoPatches(Patch P, Patch Q, int f);

    };

}

#endif //PBRT_CORE_NLMEANSFILTER_H