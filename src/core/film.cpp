
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


// core/film.cpp*
#include "film.h"
#include "paramset.h"
#include "imageio.h"
#include "stats.h"

namespace pbrt {

STAT_MEMORY_COUNTER("Memory/Film pixels", filmPixelMemory);

// Film Method Definitions
Film::Film(const Point2i &resolution, const Bounds2f &cropWindow,
           std::unique_ptr<Filter> filt, Float diagonal,
           const std::string &filename, Float scale, Float maxSampleLuminance)
    : fullResolution(resolution),
      diagonal(diagonal * .001),
      filter(std::move(filt)),
      filename(filename),
      scale(scale),
      maxSampleLuminance(maxSampleLuminance) {
    // Compute film image bounds
    croppedPixelBounds =
        Bounds2i(Point2i(std::ceil(fullResolution.x * cropWindow.pMin.x),
                         std::ceil(fullResolution.y * cropWindow.pMin.y)),
                 Point2i(std::ceil(fullResolution.x * cropWindow.pMax.x),
                         std::ceil(fullResolution.y * cropWindow.pMax.y)));
    LOG(INFO) << "Created film with full resolution " << resolution <<
        ". Crop window of " << cropWindow << " -> croppedPixelBounds " <<
        croppedPixelBounds;

    // Allocate film image storage
    SetBuffers(1);

    // Precompute filter weight table
    int offset = 0;
    for (int y = 0; y < filterTableWidth; ++y) {
        for (int x = 0; x < filterTableWidth; ++x, ++offset) {
            Point2f p;
            p.x = (x + 0.5f) * filter->radius.x / filterTableWidth;
            p.y = (y + 0.5f) * filter->radius.y / filterTableWidth;
            filterTable[offset] = filter->Evaluate(p);
        }
    }
}

Bounds2i Film::GetSampleBounds() const {
    Bounds2f floatBounds(Floor(Point2f(croppedPixelBounds.pMin) +
                               Vector2f(0.5f, 0.5f) - filter->radius),
                         Ceil(Point2f(croppedPixelBounds.pMax) -
                              Vector2f(0.5f, 0.5f) + filter->radius));
    return (Bounds2i)floatBounds;
}

Bounds2f Film::GetPhysicalExtent() const {
    Float aspect = (Float)fullResolution.y / (Float)fullResolution.x;
    Float x = std::sqrt(diagonal * diagonal / (1 + aspect * aspect));
    Float y = aspect * x;
    return Bounds2f(Point2f(-x / 2, -y / 2), Point2f(x / 2, y / 2));
}

std::unique_ptr<FilmTile> Film::GetFilmTile(const Bounds2i &sampleBounds) 
{
    // Bound image pixels that samples in _sampleBounds_ contribute to
    Vector2f halfPixel = Vector2f(0.5f, 0.5f);
    Bounds2f floatBounds = (Bounds2f)sampleBounds;
    Point2i p0 = (Point2i)Ceil(floatBounds.pMin - halfPixel - filter->radius);
    Point2i p1 = (Point2i)Floor(floatBounds.pMax - halfPixel + filter->radius) + Point2i(1, 1);
    Bounds2i tilePixelBounds = Intersect(Bounds2i(p0, p1), croppedPixelBounds);

    std::unique_ptr<FilmTile> filmTile ( new FilmTile(tilePixelBounds, filter->radius, filterTable, filterTableWidth, maxSampleLuminance, amountOfBuffers) );

    //Initialize filmTile's pixel with some values from the film's pixel to keep counting variance and mean
    for (int buffer = 0; buffer < amountOfBuffers; buffer++)
        for (Point2i pixel : filmTile->GetPixelBounds())
        {
            FilmTilePixel &tilePixel = filmTile->GetPixel(buffer, pixel);
            Pixel &filmPixel = GetPixel(buffer, pixel);

            tilePixel.previousFilterWeightSum = filmPixel.filterWeightSum;
            for (int i = 0; i < 3; i++)
            {
                tilePixel.mean[i] = filmPixel.mean[i];
                tilePixel.varianceSum[i] = filmPixel.varianceSum[i];
            }
        }

    return filmTile;
}

void Film::Clear() 
{
    for (int buffer = 0; buffer < amountOfBuffers; buffer++)
        for (Point2i p : croppedPixelBounds) 
        {
            Pixel &pixel = GetPixel(buffer, p);
            for (int c = 0; c < 3; ++c) pixel.splatXYZ[c] = pixel.xyz[c] = 0;
            pixel.filterWeightSum = 0;
        }
}

void Film::SetBuffers(const int count)
{
    CHECK(count >= 1);

    while (buffers.size() > count)
    {
        buffers.pop_back();
        filmPixelMemory -= croppedPixelBounds.Area() * sizeof(Pixel);
    }

    while (buffers.size() < count)
    {
        buffers.push_back(std::unique_ptr<Pixel[]>(new Pixel[croppedPixelBounds.Area()]));
        filmPixelMemory += croppedPixelBounds.Area() * sizeof(Pixel);
    }

    amountOfBuffers = buffers.size();
}

void Film::MergeFilmTile(std::unique_ptr<FilmTile> tile) 
{
    ProfilePhase p(Prof::MergeFilmTile);
    VLOG(1) << "Merging film tile " << tile->pixelBounds;

    std::lock_guard<std::mutex> lock(mutex);

    for (int buffer = 0; buffer < amountOfBuffers; buffer++)
        for (Point2i position : tile->GetPixelBounds()) 
        {
            // Merge _pixel_ into _Film::buffers_
            const FilmTilePixel &tilePixel = tile->GetPixel(buffer, position);

            //Can happen, due to bounds being somehow wrong, the error seems to get introduced by GetFilmTile method, where the bounds are made bigger than seemingly necessary.
            //This results in pixels being merged twice. The second time without any information in them, overwriting the valid information from before.
            if (tilePixel.filterWeightSum < 0.001) continue; //TODO:: Should not happen but does

            Pixel &mergePixel = GetPixel(buffer, position);
            Float xyz[3];
            tilePixel.contribSum.ToXYZ(xyz);
            
            //Add sample
            for (int i = 0; i < 3; ++i) mergePixel.xyz[i] += xyz[i];
            mergePixel.filterWeightSum += tilePixel.filterWeightSum;

            //Take over new mean and variance
            for (int i = 0; i < 3; i++)
            {
                mergePixel.mean[i] = tilePixel.mean[i];
                mergePixel.varianceSum[i] = tilePixel.varianceSum[i];
            }
        }
}

void Film::SetImage(const Spectrum *img, const int buffer) const 
{
    int nPixels = croppedPixelBounds.Area();
    for (int i = 0; i < nPixels; ++i) 
    {
        Pixel &p = buffers[buffer][i];
        img[i].ToXYZ(p.xyz);
        p.filterWeightSum = 1;
        p.splatXYZ[0] = p.splatXYZ[1] = p.splatXYZ[2] = 0;
    }
}

void Film::AddSplat(const Point2f &p, Spectrum v, const int buffer) 
{
    ProfilePhase pp(Prof::SplatFilm);

    if (v.HasNaNs()) 
    {
        LOG(ERROR) << StringPrintf("Ignoring splatted spectrum with NaN values "
                                   "at (%f, %f)", p.x, p.y);
        return;
    } 
    else if (v.y() < 0.) 
    {
        LOG(ERROR) << StringPrintf("Ignoring splatted spectrum with negative "
                                   "luminance %f at (%f, %f)", v.y(), p.x, p.y);
        return;
    } 
    else if (std::isinf(v.y())) 
    {
        LOG(ERROR) << StringPrintf("Ignoring splatted spectrum with infinite "
                                   "luminance at (%f, %f)", p.x, p.y);
        return;
    }

    if (!InsideExclusive((Point2i)p, croppedPixelBounds)) return;

    if (v.y() > maxSampleLuminance) v *= maxSampleLuminance / v.y();
    Float xyz[3];
    v.ToXYZ(xyz);
    Pixel &pixel = GetPixel(buffer, (Point2i)p);
    for (int i = 0; i < 3; ++i) pixel.splatXYZ[i].Add(xyz[i]);
}

void Film::WriteVarianceImage(std::string nameOfFile, int buffer, Float splatScale)
{
    SetBuffers(amountOfBuffers + 1);
    for (Point2i position : croppedPixelBounds)
    {
        Pixel& pixel = GetPixel(amountOfBuffers - 1, position);
        for (int i = 0; i < 3; i++) pixel.xyz[i] = GetPixel(buffer, position).varianceSum[i] / GetPixel(buffer, position).filterWeightSum;
        pixel.filterWeightSum = 1;
    }
    WriteBufferImage(nameOfFile, amountOfBuffers - 1);
    SetBuffers(amountOfBuffers - 1);
}

void Film::WriteBufferDifferenceImage(std::string nameOfFile, int buffer1, int buffer2, Float splatScale)
{
    SetBuffers(amountOfBuffers + 1);
    for (Point2i position : croppedPixelBounds)
    {
        Pixel& pixel = GetPixel(amountOfBuffers - 1, position);
        for (int i = 0; i < 3; i++)
        {
            Float xyz1 = GetPixel(buffer1, position).xyz[i] / GetPixel(buffer1, position).filterWeightSum;
            Float xyz2 = GetPixel(buffer2, position).xyz[i] / GetPixel(buffer2, position).filterWeightSum;
            pixel.xyz[i] = std::abs(xyz1 - xyz2);
        }
        pixel.filterWeightSum = 1;
    }
    WriteBufferImage(nameOfFile, amountOfBuffers - 1);
    SetBuffers(amountOfBuffers - 1);
}

void Film::WriteBufferImage(std::string nameOfFile, int buffer, Float splatScale)
{
    std::unique_ptr<Float[]> rgb(new Float[3 * croppedPixelBounds.Area()]);

    int offset = 0;
    for (Point2i p : croppedPixelBounds)
    {
        Pixel &pixel = GetPixel(buffer, p);
        std::shared_ptr<Pixel> sharedPixel = std::make_shared<Pixel>();

        for (int i = 0; i < 3; ++i) sharedPixel->xyz[i] = pixel.xyz[i];
        for (int i = 0; i < 3; ++i) sharedPixel->splatXYZ[i].Add(pixel.splatXYZ[i]);
        sharedPixel->filterWeightSum = pixel.filterWeightSum;
        sharedPixel->pad = pixel.pad;

        StorePixelInRGB(sharedPixel, offset, &rgb[0], splatScale);
        ++offset;
    }
    pbrt::WriteImage(nameOfFile, &rgb[0], croppedPixelBounds, fullResolution);
}

void Film::WriteImage(Float splatScale) 
{
    // Convert image to RGB and compute final pixel values
    LOG(INFO) << "Converting image to RGB and computing final weighted pixel values";
    std::unique_ptr<Float[]> rgb(new Float[3 * croppedPixelBounds.Area()]);

    int offset = 0;
    for (Point2i p : croppedPixelBounds) 
    {
        std::shared_ptr<Pixel> pixel = GetCombinedPixel(p);
        StorePixelInRGB(pixel, offset, &rgb[0], splatScale);
        ++offset;
    }

    // Write RGB image
    LOG(INFO) << "Writing image " << filename << " with bounds " << croppedPixelBounds;
    pbrt::WriteImage(filename, &rgb[0], croppedPixelBounds, fullResolution);
}

void Film::StorePixelInRGB(std::shared_ptr<Pixel> pixel, int offset, Float* rgb, Float splatScale)
{
    // Convert pixel XYZ color to RGB
    XYZToRGB(pixel->xyz, &rgb[3 * offset]);

    // Normalize pixel with weight sum
    Float filterWeightSum = pixel->filterWeightSum;
    if (filterWeightSum != 0)
    {
        Float invWt = (Float)1 / filterWeightSum;
        rgb[3 * offset] = std::max((Float)0, rgb[3 * offset] * invWt);
        rgb[3 * offset + 1] = std::max((Float)0, rgb[3 * offset + 1] * invWt);
        rgb[3 * offset + 2] = std::max((Float)0, rgb[3 * offset + 2] * invWt);
    }

    // Add splat value at pixel
    Float splatRGB[3];
    Float splatXYZ[3] = { pixel->splatXYZ[0], pixel->splatXYZ[1], pixel->splatXYZ[2] };
    XYZToRGB(splatXYZ, splatRGB);
    rgb[3 * offset] += splatScale * splatRGB[0];
    rgb[3 * offset + 1] += splatScale * splatRGB[1];
    rgb[3 * offset + 2] += splatScale * splatRGB[2];

    // Scale pixel value by _scale_
    rgb[3 * offset] *= scale;
    rgb[3 * offset + 1] *= scale;
    rgb[3 * offset + 2] *= scale;
}

Film *CreateFilm(const ParamSet &params, std::unique_ptr<Filter> filter) {
    // Intentionally use FindOneString() rather than FindOneFilename() here
    // so that the rendered image is left in the working directory, rather
    // than the directory the scene file lives in.
    std::string filename = params.FindOneString("filename", "");
    if (PbrtOptions.imageFile != "") {
        if (filename != "") {
            Warning(
                "Output filename supplied on command line, \"%s\", ignored "
                "due to filename provided in scene description file, \"%s\".",
                PbrtOptions.imageFile.c_str(), filename.c_str());
        } else
            filename = PbrtOptions.imageFile;
    }
    if (filename == "") filename = "pbrt.exr";

    int xres = params.FindOneInt("xresolution", 1280);
    int yres = params.FindOneInt("yresolution", 720);
    if (PbrtOptions.quickRender) xres = std::max(1, xres / 4);
    if (PbrtOptions.quickRender) yres = std::max(1, yres / 4);
    Bounds2f crop(Point2f(0, 0), Point2f(1, 1));
    int cwi;
    const Float *cr = params.FindFloat("cropwindow", &cwi);
    if (cr && cwi == 4) {
        crop.pMin.x = Clamp(std::min(cr[0], cr[1]), 0.f, 1.f);
        crop.pMax.x = Clamp(std::max(cr[0], cr[1]), 0.f, 1.f);
        crop.pMin.y = Clamp(std::min(cr[2], cr[3]), 0.f, 1.f);
        crop.pMax.y = Clamp(std::max(cr[2], cr[3]), 0.f, 1.f);
    } else if (cr)
        Error("%d values supplied for \"cropwindow\". Expected 4.", cwi);

    Float scale = params.FindOneFloat("scale", 1.);
    Float diagonal = params.FindOneFloat("diagonal", 35.);
    Float maxSampleLuminance = params.FindOneFloat("maxsampleluminance",
                                                   Infinity);
    return new Film(Point2i(xres, yres), crop, std::move(filter), diagonal,
                    filename, scale, maxSampleLuminance);
}

}  // namespace pbrt
