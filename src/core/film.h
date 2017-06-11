
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_FILM_H
#define PBRT_CORE_FILM_H

// core/film.h*
#include "pbrt.h"
#include "geometry.h"
#include "spectrum.h"
#include "filter.h"
#include "stats.h"
#include "parallel.h"

namespace pbrt 
{

    struct Pixel 
    {
        Float xyz[3];
        Float filterWeightSum;
        AtomicFloat splatXYZ[3];
        Float pad;

        Pixel() { xyz[0] = xyz[1] = xyz[2] = filterWeightSum = 0; }
    };

    struct FilmTilePixel 
    {
        Spectrum contribSum = 0.f;
        Float filterWeightSum = 0.f;
    };
    
    // Film Declarations
    class Film 
    {
      public:
          // Film Public Data
          const Point2i fullResolution;
          const Float diagonal;
          std::unique_ptr<Filter> filter;
          const std::string filename;
          Bounds2i croppedPixelBounds;
          int amountOfBuffers;

          // Film Public Methods
          Film(const Point2i &resolution, const Bounds2f &cropWindow,
               std::unique_ptr<Filter> filter, Float diagonal,
               const std::string &filename, Float scale,
               Float maxSampleLuminance = Infinity);

          Bounds2i GetSampleBounds() const;
          Bounds2f GetPhysicalExtent() const;
          std::unique_ptr<FilmTile> GetFilmTile(const Bounds2i &sampleBounds);
          void MergeFilmTile(std::unique_ptr<FilmTile> tile);
          void SetImage(const Spectrum *img, const int buffer = 0) const;
          void AddSplat(const Point2f &p, Spectrum v, const int buffer = 0); //Used in bdpt and mlt integrators! Maybe it should distribute splats over the buffers instead of defaulting to the first, as with the FilmTiles AddSample method?
          void WriteImage(Float splatScale = 1);
          void Clear();
          void SetBuffers(const int count = 1);
    
          Pixel &GetPixel(const int buffer, const Point2i &p)
          {
              CHECK(buffer >= 0 && buffer < buffers.size());
              CHECK(InsideExclusive(p, croppedPixelBounds));

              int width = croppedPixelBounds.pMax.x - croppedPixelBounds.pMin.x;
              int offset = (p.x - croppedPixelBounds.pMin.x) + (p.y - croppedPixelBounds.pMin.y) * width;
              return buffers[buffer][offset];
          }

          std::shared_ptr<Pixel> GetCombinedPixel(const Point2i &p)
          {
              CHECK(InsideExclusive(p, croppedPixelBounds));

              std::shared_ptr<Pixel> combined = std::make_shared<Pixel>();
              for (int buffer = 0; buffer < amountOfBuffers; buffer++)
              {
                  Pixel &merge = GetPixel(buffer, p);

                  for (int i = 0; i < 3; ++i) combined->xyz[i] += merge.xyz[i];
                  for (int i = 0; i < 3; ++i) combined->splatXYZ[i].Add(merge.splatXYZ[i]);
                  combined->filterWeightSum += merge.filterWeightSum;
                  combined->pad = merge.pad;
              }

              return combined;
          }

      private:
          // Film Private Data
          std::vector<std::unique_ptr<Pixel[]>> buffers;
          static PBRT_CONSTEXPR int filterTableWidth = 16;
          Float filterTable[filterTableWidth * filterTableWidth];
          std::mutex mutex;
          const Float scale;
          const Float maxSampleLuminance;
    };
    
    class FilmTile 
    {
      public:
        // FilmTile Public Methods
        FilmTile(const Bounds2i &pixelBounds, const Vector2f &filterRadius, const Float *filterTable,
                 int filterTableSize, Float maxSampleLuminance, const int amountOfBuffers)
            : pixelBounds(pixelBounds),
              filterRadius(filterRadius),
              invFilterRadius(1 / filterRadius.x, 1 / filterRadius.y),
              filterTable(filterTable),
              filterTableSize(filterTableSize),
              maxSampleLuminance(maxSampleLuminance) 
        {
            for (int buffer = 0; buffer < amountOfBuffers; buffer++)
            {
                std::vector<FilmTilePixel> currentBuffer = std::vector<FilmTilePixel>(std::max(0, pixelBounds.Area()));
                buffers.push_back(currentBuffer);
            }
        }
    
        void AddSample(const Point2f &pFilm, Spectrum L, Float sampleWeight = 1.) 
        {
            ProfilePhase _(Prof::AddFilmSample);

            if (L.y() > maxSampleLuminance) L *= maxSampleLuminance / L.y();
            // Compute sample's raster bounds
            Point2f pFilmDiscrete = pFilm - Vector2f(0.5f, 0.5f);
            Point2i p0 = (Point2i)Ceil(pFilmDiscrete - filterRadius);
            Point2i p1 = (Point2i)Floor(pFilmDiscrete + filterRadius) + Point2i(1, 1);
            p0 = Max(p0, pixelBounds.pMin);
            p1 = Min(p1, pixelBounds.pMax);
    
            // Loop over filter support and add sample to pixel arrays
    
            // Precompute $x$ and $y$ filter table offsets
            int *ifx = ALLOCA(int, p1.x - p0.x);
            for (int x = p0.x; x < p1.x; ++x) 
            {
                Float fx = std::abs((x - pFilmDiscrete.x) * invFilterRadius.x * filterTableSize);
                ifx[x - p0.x] = std::min((int)std::floor(fx), filterTableSize - 1);
            }
    
            int *ify = ALLOCA(int, p1.y - p0.y);
            for (int y = p0.y; y < p1.y; ++y) 
            {
                Float fy = std::abs((y - pFilmDiscrete.y) * invFilterRadius.y * filterTableSize);
                ify[y - p0.y] = std::min((int)std::floor(fy), filterTableSize - 1);
            }
    
            for (int y = p0.y; y < p1.y; ++y) 
                for (int x = p0.x; x < p1.x; ++x) 
                {
                    // Evaluate filter value at $(x,y)$ pixel
                    int offset = ify[y - p0.y] * filterTableSize + ifx[x - p0.x];
                    Float filterWeight = filterTable[offset];
    
                    // Update pixel values with filtered sample contribution
                    FilmTilePixel &pixel = GetPixel(addToBufferIndex, Point2i(x, y));
                    pixel.contribSum += L * sampleWeight * filterWeight;
                    pixel.filterWeightSum += filterWeight;

                    IncrementBufferIndex();
                }
        }
    
        FilmTilePixel &GetPixel(const int buffer, const Point2i &p) 
        {
            CHECK(buffer >= 0 && buffer < buffers.size());
            CHECK(InsideExclusive(p, pixelBounds));

            int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
            int offset = (p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
            return buffers[buffer][offset];
        }
    
        const FilmTilePixel &GetPixel(const int buffer, const Point2i &p) const 
        {
            CHECK(buffer >= 0 && buffer < buffers.size());
            CHECK(InsideExclusive(p, pixelBounds));

            int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
            int offset = (p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
            return buffers[buffer][offset];
        }
    
        Bounds2i GetPixelBounds() const { return pixelBounds; }
    
      private:
        // FilmTile Private Data
        const Bounds2i pixelBounds;
        const Vector2f filterRadius, invFilterRadius;
        const Float *filterTable;
        const int filterTableSize;
        std::vector<std::vector<FilmTilePixel>> buffers;
        const Float maxSampleLuminance;
        friend class Film;
        int addToBufferIndex = 0;

        void IncrementBufferIndex()
        {
            addToBufferIndex += 1;
            if (addToBufferIndex >= buffers.size()) addToBufferIndex = 0;
        }
    };
    
    Film *CreateFilm(const ParamSet &params, std::unique_ptr<Filter> filter);

}  // namespace pbrt

#endif  // PBRT_CORE_FILM_H
