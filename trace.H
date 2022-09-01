#ifndef TRACE_H
#define TRACE_H

#include "slVector.H"
#include <vector>

class Ray {
public:
    SlVector3 e;
    SlVector3 d;
    int depth;
    Ray(const SlVector3 &_e, const SlVector3 &_d, int _depth = 0) : e(_e), d(_d), depth(_depth) {};
};

class Fill {
public: 
    SlVector3 color;
    double kd, ks, shine, t, ior;
};

class HitRecord {
public:
    double t = std::numeric_limits<double>::max(), alpha, beta, gamma;
    SlVector3 p, n, v;
    Fill f;
    int raydepth;
};

class Light {
public:
    SlVector3 p, c;
};

struct AABB
{
public:
    SlVector3 boundMin;
    SlVector3 boundMax;

    AABB() : boundMin(std::numeric_limits<double>::max()), boundMax(std::numeric_limits<double>::lowest())
    {

    }

    AABB(const SlVector3& _min, const SlVector3& _max) : boundMin(_min), boundMax(_max)
    {

    }

    bool intersect(const Ray& r) const
    {
        double tmin = std::numeric_limits<double>::lowest();
        double tmax = std::numeric_limits<double>::max();
        if(r.d.x() != 0.0)
        {

            double tx1 = (boundMin.x() - r.e.x()) / r.d.x();
            double tx2 = (boundMax.x() - r.e.x()) / r.d.x();
            tmin = std::max(tmin, std::min(tx1, tx2));
            tmax = std::min(tmax, std::max(tx1, tx2));
        }

        if(r.d.y() != 0.0)
        {
            double ty1 = (boundMin.y() - r.e.y()) / r.d.y();
            double ty2 = (boundMax.y() - r.e.y()) / r.d.y();

            tmin = std::max(tmin, std::min(ty1, ty2));
            tmax = std::min(tmax, std::max(ty1, ty2));
        }
        
        if(r.d.z() != 0.0)
        {
            double tz1 = (boundMin.z() - r.e.z()) / r.d.z();
            double tz2 = (boundMax.z() - r.e.z()) / r.d.z();

            tmin = std::max(tmin, std::min(tz1, tz2));
            tmax = std::min(tmax, std::max(tz1, tz2));
        }


        return tmin <= tmax;  
    }

    SlVector3 extent() const
    {
        return boundMax - boundMin;
    }
};

class Surface {
public:
    virtual bool intersect(const Ray &r, double t0, double t1, HitRecord &hr) const = 0;
    virtual AABB buildAABB() const = 0;
    virtual SlVector3 centroid() const = 0;
    virtual ~Surface() {};
};

class Triangle : public Surface {
    SlVector3 a,b,c;
public:
    Triangle(const SlVector3 &_a, const SlVector3 &_b, const SlVector3 &_c) : a(_a), b(_b), c(_c) {};
    virtual bool intersect(const Ray &r, double t0, double t1, HitRecord &hr) const override;

    inline virtual AABB buildAABB() const override
    {
        SlVector3 boundMin(std::numeric_limits<double>::max());
        SlVector3 boundMax(std::numeric_limits<double>::lowest());

        boundMin.minSet(a);
        boundMin.minSet(b);
        boundMin.minSet(c);

        boundMax.maxSet(a);
        boundMax.maxSet(b);
        boundMax.maxSet(c);
        return AABB{boundMin, boundMax};
    }

    inline virtual SlVector3 centroid() const override
    {
        return (a + b + c) / 3.0;
    }
};

class TrianglePatch : public Triangle {
    SlVector3 n1, n2, n3;
public: 
    TrianglePatch(const SlVector3 &_a, const SlVector3 &_b, const SlVector3 &_c,
    const SlVector3 &_n1, const SlVector3 &_n2, const SlVector3 &_n3) 
        : Triangle(_a,_b,_c), n1(_n1), n2(_n2), n3(_n3) {};
    virtual bool intersect(const Ray &r, double t0, double t1, HitRecord &hr) const override;
};

class Sphere : public Surface {
    SlVector3 c;
    double rad;
public:
    Sphere(const SlVector3 &_c, double _r) : c(_c), rad(_r) {};
    bool intersect(const Ray &r, double t0, double t1, HitRecord &hr) const override;

    inline virtual AABB buildAABB() const override
    {
        SlVector3 boundMin(std::numeric_limits<double>::max());
        SlVector3 boundMax(std::numeric_limits<double>::lowest());

        boundMin.minSet(c - SlVector3{rad, rad, rad});
        boundMax.maxSet(c + SlVector3{rad, rad, rad});

        return AABB{boundMax, boundMax};
    }

    inline virtual SlVector3 centroid() const override
    {
        return c;
    }
};

class Tracer {
    SlVector3 bcolor, eye, at, up;
    double angle, hither;
    unsigned int res[2];
    std::vector<std::pair<Surface *, Fill> > surfaces;
    std::vector<Light> lights;
    double shadowbias;
  
    SlVector3 *im;
public:
    Tracer(const std::string &fname);
    ~Tracer();
    void traceImage();  
    SlVector3 trace(const Ray &ray, double t0, double t1) const;
    SlVector3 shade(const HitRecord &hr) const;
    void writeImage(const std::string &fname);
    void traceTask(uint32_t i, uint32_t j, uint32_t samples, SlVector3* pixel);

    bool color;
    int samples;
    double aperture;
    int maxraydepth;
};
	
#endif
