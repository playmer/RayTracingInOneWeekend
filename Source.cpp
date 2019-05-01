#include <cinttypes>
#include <vector>
#include <cstdint>
#include <memory>
#include <limits>
#include <random>
#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "glm/glm.hpp"

struct Pixel
{
  Pixel(uint8_t aR, uint8_t aG, uint8_t aB)
    : r{ aR }
    , g{ aG }
    , b{ aB }
    , a{ 255 }
  {

  }

  Pixel(glm::vec3 const& aColor)
    : r{ static_cast<uint8_t>(255.99f * aColor.r) }
    , g{ static_cast<uint8_t>(255.99f * aColor.g) }
    , b{ static_cast<uint8_t>(255.99f * aColor.b) }
    , a{ 255 }
  {

  }

  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
};

class Random
{
public: 
  Random()
    : mDistribution{0, 1}
  {
    mEngine.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }

  // [0, 1)
  double GetRandom()
  {
    return mDistribution(mEngine);
  }
  std::default_random_engine mEngine;
  std::uniform_real_distribution<double> mDistribution;
};

namespace PathTracing
{
  using namespace glm;

  Random gRandom;

  struct Ray
  {
    Ray() {}

    Ray(vec3 const& a, vec3 const& b)
      : mA{ a }
      , mB{ b }
    {

    }

    vec3 const& Origin() const { return mA; }
    vec3 const& Direction() const { return mB; }
    vec3 PointAtT(float t) const
    {
      return mA + (t * mB);
    }


    vec3 mA;
    vec3 mB;
  };

  vec3 RandomInUnitSphere()
  {
    vec3 p;
    float pLength{ 0.0f };

    do
    {
      p = (2.0f * vec3(gRandom.GetRandom(), gRandom.GetRandom(), gRandom.GetRandom())) - vec3(1, 1, 1);
      pLength = length(p);
    } while ((pLength * pLength) >= 1.0f);

    return p;
  }

  class Camera
  {
  public:
    Camera()
      : mOrigin{ 0, 0, 0 }
      , mLowerLeftCorner{ -2, -1, -1 }
      , mHorizontal{ 4, 0, 0 }
      , mVertical{ 0, 2, 0 }
    {

    }

    Ray GetRay(float aU, float aV)
    {
      return Ray{ mOrigin, mLowerLeftCorner + (aU * mHorizontal) + (aV * mVertical) };
    }

    vec3 mOrigin;
    vec3 mLowerLeftCorner;
    vec3 mHorizontal;
    vec3 mVertical;
  };

  struct HitRecord
  {
    float t;
    vec3 p;
    vec3 normal;
  };

  class Hitable
  {
  public:
    virtual bool hit(Ray const& r, float aTMin, float aTMax, HitRecord& aRecord) const = 0;
  };


  class Sphere : public Hitable
  {
  public:
    Sphere(vec3 const& aCenter, float aRadius)
      : mCenter{aCenter}
      , mRadius {aRadius}
    {

    }

    bool hit(Ray const& aRay, float aTMin, float aTMax, HitRecord& aRecord) const override
    {
      vec3 oc = aRay.Origin() - mCenter;

      float a = dot(aRay.Direction(), aRay.Direction());
      float b = dot(oc, aRay.Direction());
      float c = dot(oc, oc) - (mRadius * mRadius);
      float discriminant = (b * b) - (a * c);

      if (discriminant > 0)
      {
        float temp = (-b - sqrt(discriminant)) / a;

        if ((aTMin < temp) && (temp < aTMax))
        {
          aRecord.t = temp;
          aRecord.p = aRay.PointAtT(temp);
          aRecord.normal = (aRecord.p - mCenter) / mRadius;
          return true;
        }

        temp = (-b + sqrt(discriminant)) / a;

        if ((aTMin < temp) && (temp < aTMax))
        {
          aRecord.t = temp;
          aRecord.p = aRay.PointAtT(temp);
          aRecord.normal = (aRecord.p - mCenter) / mRadius;
          return true;
        }
      }

      return false;
    }

  private:
    vec3 mCenter;
    float mRadius;
  };


  class World : public Hitable
  {
  public:


    bool hit(Ray const& aRay, float aTMin, float aTMax, HitRecord& aRecord) const override
    {
      bool hit{ false };
      double closestHit = aTMax;
      HitRecord temp;

      for (auto const& hitable : mHitables)
      {
        if (hitable->hit(aRay, aTMin, closestHit, temp))
        {
          aRecord = temp;
          closestHit = aRecord.t;
          hit = true;
        }
      }

      return hit;
    }

    template <typename tType, typename ...tArguments>
    void Add(tArguments&&... aArguments)
    {
      mHitables.emplace_back(std::make_unique<tType>(std::forward<tArguments>(aArguments)...));
    }

    std::vector<std::unique_ptr<Hitable>> mHitables;
  };



  vec3 Color(Ray const& aRay, World const& aWorld)
  {
    HitRecord record;

    if (aWorld.hit(aRay, 0.0f, std::numeric_limits<float>::max(), record))
    {
      vec3 target = record.p + record.normal + RandomInUnitSphere();
      return 0.5f* Color(Ray{record.p, target - record.p}, aWorld);
    }
    else
    {
      auto direction = glm::normalize(aRay.Direction());

      float t = 0.5f * (direction.y + 1.0f);
      return ((1.0f - t) * glm::vec3{ 1.0f, 1.0f, 1.0f }) + (t * glm::vec3{ 0.5f, 0.7f, 1.0f });
    }
  }
}

static float gComplete = 0.0f;

std::vector<Pixel> RenderFrame(size_t aWidth, size_t aHeight, size_t aRaysPerPixel)
{
  Random random;
  std::vector<Pixel> pixels;

  size_t totalPixels = aWidth * aHeight;
  size_t pixelsComplete = 0;

  pixels.reserve(totalPixels);

  float fWidth = static_cast<float>(aWidth);
  float fHeight = static_cast<float>(aHeight);

  PathTracing::Camera camera;

  PathTracing::World world;

  world.Add<PathTracing::Sphere>(glm::vec3{ 0,0,-1 }, 0.5f);
  world.Add<PathTracing::Sphere>(glm::vec3{ 0,-100.5,-1 }, 100.f);

  for (std::int64_t j = static_cast<std::int64_t>(aHeight - 1); j >= 0; --j)
  {
    for (size_t i = 0; i < aWidth; ++i)
    {
      glm::vec3 color{0,0,0};

      for (size_t s = 0; s < aRaysPerPixel; ++s)
      {
        float u = (i + random.GetRandom()) / fWidth;
        float v = (j + random.GetRandom()) / fHeight;
        
        auto ray = camera.GetRay(u, v);
        color += PathTracing::Color(ray, world);
      }

      color /= aRaysPerPixel;

      pixels.emplace_back(color);

      ++pixelsComplete;

      gComplete = static_cast<float>(pixelsComplete) / totalPixels;
    }

    printf("%f\n", gComplete);
  }

  return std::move(pixels);
}

int main()
{
  size_t width = 600;
  size_t height = 300;
  auto pixels =  RenderFrame(width, height, 100);

  stbi_write_png("output.png",
                 width,
                 height,
                 4,
                 reinterpret_cast<void const*>(pixels.data()),
                 static_cast<int>(sizeof(Pixel) * width));

  return 0;
}