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

#include "ThreadPool/ThreadPool.h"

struct Pixel
{
  Pixel()
    : r{ 0 }
    , g{ 0 }
    , b{ 0 }
    , a{ 255 }
  {

  }

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

  template <typename tType>
  inline tType Square(tType aValue)
  {
    return aValue * aValue;
  }

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

    Ray GetRay(float aU, float aV) const
    {
      return Ray{ mOrigin, mLowerLeftCorner + (aU * mHorizontal) + (aV * mVertical) };
    }

    vec3 mOrigin;
    vec3 mLowerLeftCorner;
    vec3 mHorizontal;
    vec3 mVertical;
  };

  bool refract(vec3 const& v, vec3 const& n, float ni_over_nt, vec3& refracted)
  {
    vec3 uv = normalize(v);

    float dt = dot(uv, n);
    float discriminant = 1.0f - Square(ni_over_nt) * (1 - Square(dt));

    if (discriminant > 0.f)
    {
      refracted = ni_over_nt * (uv - n * dt) - n * sqrt(discriminant);
      return true;
    }
    else
    {
      return false;
    }
  }

  float Schlick(float cosine, float aRefractiveIndex)
  {
    float r0 = (1.f - aRefractiveIndex) / (1 + aRefractiveIndex);
    r0 = Square(r0);

    return r0 + (1 - r0) * pow((1 - cosine), 5);
  }

  class Material;

  struct HitRecord
  {
    float t;
    vec3 p;
    vec3 normal;
    Material* mMaterial;
  };

  class Hitable
  {
  public:
    virtual bool hit(Ray const& aRay, float aTMin, float aTMax, HitRecord& aRecord) const = 0;
  };

  class Material
  {
  public:
    virtual bool scatter(Ray const& aRay, HitRecord& aRecord, vec3& aAttenuation, Ray& aScattered) const = 0;
  };

  class Lambertian : public Material
  {
  public:
    Lambertian(vec3 aAlbedo)
      : mAlbedo{aAlbedo}
    {

    }

    bool scatter(Ray const& aRay, HitRecord& aRecord, vec3& aAttenuation, Ray& aScattered) const override
    {
      vec3 target = aRecord.p + aRecord.normal + RandomInUnitSphere();
      aScattered = Ray{ aRecord.p, target - aRecord.p };
      aAttenuation = mAlbedo;

      return true;
    }


    vec3 mAlbedo;
  };

  class Metal : public Material
  {
  public:
    Metal(vec3 aAlbedo, float aFuzz)
      : mAlbedo{ aAlbedo }
      , mFuzz{ aFuzz }
    {
      if (mFuzz > 1)
      {
        mFuzz = 1.f;
      }
    }

    bool scatter(Ray const& aRay, HitRecord& aRecord, vec3& aAttenuation, Ray& aScattered) const override
    {
      vec3 reflected = reflect(normalize(aRay.Direction()), aRecord.normal);
      aScattered = Ray{ aRecord.p, reflected + mFuzz * RandomInUnitSphere() };
      aAttenuation = mAlbedo;

      return (dot(aScattered.Direction(), aRecord.normal) > 0.f);
    }


    vec3 mAlbedo;
    float mFuzz;
  };

  class Dielectric : public Material
  {
  public:
    Dielectric(float aRefractiveIndex)
      : mRefractiveIndex{ aRefractiveIndex }
    {

    }

    bool scatter(Ray const& aRay, HitRecord& aRecord, vec3& aAttenuation, Ray& aScattered) const override
    {
      vec3 outwardNormal;
      vec3 reflected = reflect(aRay.Direction(), aRecord.normal);
      float ni_over_nt;
      aAttenuation = vec3{ 1,1,1 };
      vec3 refracted;
      float cosine;
      float reflectiveProbability = 0.0f;

      float dotP = dot(aRay.Direction(), aRecord.normal);
      cosine = dotP / length(aRay.Direction());

      if (dotP > 0)
      {
        outwardNormal = -aRecord.normal;
        ni_over_nt = mRefractiveIndex;

        cosine = sqrt(1.f - Square(mRefractiveIndex) * (1.0f - Square(cosine)));
      }
      else
      {
        outwardNormal = aRecord.normal;
        ni_over_nt = 1.0f / mRefractiveIndex;

        cosine = -cosine;
      }

      if (refract(aRay.Direction(), outwardNormal, ni_over_nt, refracted))
      {
        reflectiveProbability = Schlick(cosine, mRefractiveIndex);
      }
      else
      {
        aScattered = Ray{ aRecord.p, reflected };
        reflectiveProbability = 1.f;
      }

      if (gRandom.GetRandom() < reflectiveProbability)
      {
        aScattered = Ray{ aRecord.p, reflected };
      }
      else
      {
        aScattered = Ray{ aRecord.p, refracted };
      }

      return true;
    }

    float mRefractiveIndex;
  };

  class Sphere : public Hitable
  {
  public:
    Sphere(vec3 const& aCenter, float aRadius, std::unique_ptr<Material> aMaterial)
      : mCenter{aCenter}
      , mRadius {aRadius}
      , mMaterial{std::move(aMaterial)}
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
          aRecord.mMaterial = mMaterial.get();
          return true;
        }

        temp = (-b + sqrt(discriminant)) / a;

        if ((aTMin < temp) && (temp < aTMax))
        {
          aRecord.t = temp;
          aRecord.p = aRay.PointAtT(temp);
          aRecord.normal = (aRecord.p - mCenter) / mRadius;
          aRecord.mMaterial = mMaterial.get();
          return true;
        }
      }

      return false;
    }

  private:
    std::unique_ptr<Material> mMaterial;
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



  vec3 Color(Ray const& aRay, World const& aWorld, size_t aDepth)
  {
    HitRecord record;

    if (aWorld.hit(aRay, 0.001f, std::numeric_limits<float>::max(), record))
    {
      Ray scattered;
      vec3 attenuation;

      if (aDepth < 50 && record.mMaterial->scatter(aRay, record, attenuation, scattered))
      {
        return attenuation * Color(scattered, aWorld, aDepth + 1);
      }
      else
      {
        return vec3{ 0, 0, 0 };
      }
    }
    else
    {
      auto direction = glm::normalize(aRay.Direction());

      float t = 0.5f * (direction.y + 1.0f);
      return ((1.0f - t) * glm::vec3{ 1.0f, 1.0f, 1.0f }) + (t * glm::vec3{ 0.5f, 0.7f, 1.0f });
    }
  }

  std::vector<Pixel> RenderFrame(size_t aWidth, size_t aHeight, size_t aRaysPerPixel)
  {
    std::vector<Pixel> pixels;

    pixels.resize(aWidth * aHeight);

    float fWidth = static_cast<float>(aWidth);
    float fHeight = static_cast<float>(aHeight);

    PathTracing::Camera const camera;

    PathTracing::World world;

    world.Add<PathTracing::Sphere>(glm::vec3{ 0,0,-1 }, 0.5f, std::make_unique<Lambertian>(vec3{.1f, .2f, .5f}));
    world.Add<PathTracing::Sphere>(glm::vec3{ 0,-100.5,-1 }, 100.f, std::make_unique<Lambertian>(vec3{ .8f, .8f, .0f }));
    world.Add<PathTracing::Sphere>(glm::vec3{ 1,0,-1 }, 0.5f, std::make_unique<Metal>(vec3{ .8f, .6f, .2f }, 0.1f));
    world.Add<PathTracing::Sphere>(glm::vec3{ -1,0,-1 }, 0.5f, std::make_unique<Dielectric>(1.5f));

    progschj::ThreadPool pool{std::thread::hardware_concurrency()};

    size_t k = 0;

    for (std::int64_t j = static_cast<std::int64_t>(aHeight - 1); j >= 0; --j)
    {
      for (size_t i = 0; i < aWidth; ++i)
      {
        pool.enqueue([fWidth, fHeight, &camera, &world, aRaysPerPixel, &pixels, i, j, k]()
        {
          static thread_local Random random;
          glm::vec3 color{ 0,0,0 };

          for (size_t s = 0; s < aRaysPerPixel; ++s)
          {
            float u = (i + random.GetRandom()) / fWidth;
            float v = (j + random.GetRandom()) / fHeight;

            auto ray = camera.GetRay(u, v);
            color += PathTracing::Color(ray, world, 0);
          }

          color /= aRaysPerPixel;

          color = glm::vec3{ sqrtf(color[0]), sqrtf(color[1]), sqrtf(color[2]) };

          pixels[k] = color;
        });

        ++k;
      }
    }

    pool.wait_until_empty();
    pool.wait_until_nothing_in_flight();

    return std::move(pixels);
  }
}

int main()
{
  size_t width = 600;
  size_t height = 300;
  auto pixels =  PathTracing::RenderFrame(width, height, 100);

  stbi_write_png("output.png",
                 width,
                 height,
                 4,
                 reinterpret_cast<void const*>(pixels.data()),
                 static_cast<int>(sizeof(Pixel) * width));

  return 0;
}