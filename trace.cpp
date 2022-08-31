#include "trace.H"
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <getopt.h>
#include "tbb/blocked_range.h"
#include "tbb/blocked_range2d.h"
#include "tbb/parallel_for.h"
#include "tbb/task_scheduler_observer.h"

#ifdef __APPLE__
#define MAX std::numeric_limits<double>::max()
#else
#include <values.h>
#define MAX DBL_MAX
#endif


//////////////////////////////////////////////////////////////
/// Custom Tbb Executor For Rendering

struct RenderTask
{

}

struct RenderExecutor
{


    std::vector<RenderTask> tasks;
}



//////////////////////////////////////////////////////////////

// return the determinant of the matrix with columns a, b, c.
double det(const SlVector3 &a, const SlVector3 &b, const SlVector3 &c) {
    return a[0] * (b[1] * c[2] - c[1] * b[2]) +
        b[0] * (c[1] * a[2] - a[1] * c[2]) +
            c[0] * (a[1] * b[2] - b[1] * a[2]);
}

inline double sqr(double x) {return x*x;} 

bool Triangle::intersect(const Ray &r, double t0, double t1, HitRecord &hr) const {


    auto d = r.d;
    normalize(d);

    auto detA = det(a - b, a - c, d);

    auto t = det(a - b, a - c, a - r.e) / detA;
    if(t < t0 || t > t1) return false;

    auto _beta = det(a - r.e, a - c, d) / detA;
    if(_beta < 0 || _beta > 1) return false;

    auto _gamma = det(a - b, a - r.e, d) / detA;
    if(_gamma < 0 || _gamma > 1 - _beta) return false;

    hr.alpha = 1;
    hr.beta = _beta;
    hr.gamma = _gamma;
    hr.t = t;

    // Setup hitpoint
    hr.p = t * d + r.e;
    // Setup & Normalize View Vector
    hr.v = r.e - hr.p;
    normalize(hr.v);

    auto n1 = cross(b - a, c - a);
    normalize(n1);

    auto n2 = cross(c - b, a - b);
    normalize(n2);

    auto n3 = cross(a - c, b - c);
    normalize(n3);

    hr.n = hr.alpha * n1 + hr.beta * n2 + hr.gamma * n3;
    normalize(hr.n);

    return true;
}

bool TrianglePatch::intersect(const Ray &r, double t0, double t1, HitRecord &hr) const {
    bool temp = Triangle::intersect(r,t0,t1,hr);
    if (temp) {
        hr.n = hr.alpha * n1 + hr.beta * n2 + hr.gamma * n3;
        normalize(hr.n);
    }
    return temp;
}


bool Sphere::intersect(const Ray &r, double t0, double t1, HitRecord &hr) const {

    // Intersection occurs when:
    // ||d * t + e - center|| == radius
    // (d * t + e - center) * (d * t + e - center) == radius * radius
    // t^2 <d, d> + 2t <d, e - center> + <e - center, e- center>  - radius * radius = 0

    auto _a = dot(r.d, r.d);
    auto _b = 2 * dot(r.d, r.e - c);
    auto _c = dot(r.e - c, r.e - c) - rad * rad;

    auto delta = _b * _b - 4 * _a * _c;

    // No solution to intersection equation -> no intersection
    if(delta < 0) return false;

    auto t = (-_b - sqrt(delta)) / (2 * _a);
    if(t < 0)  t = (-_b + sqrt(delta)) / (2 * _a);

    if(t < 0 || t < t0 || t > t1) return false;

    hr.t = t;
    // Setup hit point
    hr.p = t * r.d + r.e;

    // Setup normal vector
    hr.n = hr.p - c;
    normalize(hr.n);

    // Setup view vector
    hr.v = r.e - hr.p;
    normalize(hr.v);
    
    return true;
}


Tracer::Tracer(const std::string &fname) {
    std::ifstream in(fname.c_str(), std::ios_base::in);
    std::string line;
    char ch;
    Fill fill;
    bool coloredlights = false;
    while (in) {
        getline(in, line);
        switch (line[0]) {
            case 'b': {
                std::stringstream ss(line);
                ss>>ch>>bcolor[0]>>bcolor[1]>>bcolor[2];
                break;
            }

            case 'v': {
                getline(in, line);
                std::string junk;
                std::stringstream fromss(line);
                fromss>>junk>>eye[0]>>eye[1]>>eye[2];

                getline(in, line);
                std::stringstream atss(line);
                atss>>junk>>at[0]>>at[1]>>at[2];

                getline(in, line);
                std::stringstream upss(line);
                upss>>junk>>up[0]>>up[1]>>up[2];

                getline(in, line);
                std::stringstream angless(line);
                angless>>junk>>angle;

                getline(in, line);
                std::stringstream hitherss(line);
                hitherss>>junk>>hither;

                getline(in, line);
                std::stringstream resolutionss(line);
                resolutionss>>junk>>res[0]>>res[1];
                break;
            }

            case 'p': {
                bool patch = false;
                std::stringstream ssn(line);
                unsigned int nverts;
                if (line[1] == 'p') {
                    patch = true;
                    ssn>>ch;
                }
                ssn>>ch>>nverts;
                std::vector<SlVector3> vertices;
                std::vector<SlVector3> normals;
                for (unsigned int i=0; i<nverts; i++) {
                    getline(in, line);
                    std::stringstream ss(line);
                    SlVector3 v,n;
                    if (patch) ss>>v[0]>>v[1]>>v[2]>>n[0]>>n[1]>>n[2];
                    else ss>>v[0]>>v[1]>>v[2];
                    vertices.push_back(v);
                    normals.push_back(n);
                }
                bool makeTriangles = false;
                if (vertices.size() == 3) {
                    if (patch) {
                        surfaces.push_back(std::pair<Surface *, Fill>(new TrianglePatch(vertices[0], vertices[1], vertices[2], 
                        normals [0], normals [1], normals [2]), fill));
                    } else {
                        surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2]), fill));
                    }
                } else if (vertices.size() == 4) {
                    SlVector3 n0 = cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
                    SlVector3 n1 = cross(vertices[2] - vertices[1], vertices[3] - vertices[1]);
                    SlVector3 n2 = cross(vertices[3] - vertices[2], vertices[0] - vertices[2]);
                    SlVector3 n3 = cross(vertices[0] - vertices[3], vertices[1] - vertices[3]);
                    if (dot(n0,n1) > 0 && dot(n0,n2) > 0 && dot(n0,n3) > 0) {
                        makeTriangles = true;
                        if (patch) {
                            surfaces.push_back(std::pair<Surface *, Fill>(new TrianglePatch(vertices[0], vertices[1], vertices[2], 
                            normals[0], normals[1], normals[2]), fill));
                            surfaces.push_back(std::pair<Surface *, Fill>(new TrianglePatch(vertices[0], vertices[2], vertices[3], 
                            normals[0], normals[2], normals[3]), fill));
                        } else {
                            surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2]), fill));
                            surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[2], vertices[3]), fill));
                        }
                    }
                    if (!makeTriangles) {
                        std::cerr << "I didn't make triangles.  Poly not flat or more than quad.\n";
                    }
                }
                break;
            }

            case 's' : {
                std::stringstream ss(line);
                SlVector3 c;
                double r;
                ss>>ch>>c[0]>>c[1]>>c[2]>>r;
                surfaces.push_back(std::pair<Surface *, Fill>(new Sphere(c,r), fill));
                break;
            }
	  
            case 'f' : {
                std::stringstream ss(line);
                ss>>ch>>fill.color[0]>>fill.color[1]>>fill.color[2]>>fill.kd>>fill.ks>>fill.shine>>fill.t>>fill.ior;
                break;
            }

            case 'l' : {
                std::stringstream ss(line);
                Light l;
                ss>>ch>>l.p[0]>>l.p[1]>>l.p[2];
                if (!ss.eof()) {
                    ss>>l.c[0]>>l.c[1]>>l.c[2];
                    coloredlights = true;
                }
                lights.push_back(l);
                break;
            }

            default:
            break;
        }
    }
    if (!coloredlights) for (unsigned int i=0; i<lights.size(); i++) lights[i].c = 1.0/sqrt(lights.size());
    im = new SlVector3[res[0]*res[1]];
    shadowbias = 1e-6;
    samples = 1;
    aperture = 0.0;
}

Tracer::~Tracer() {
    if (im) delete [] im;
    for (unsigned int i=0; i<surfaces.size(); i++) delete surfaces[i].first;
}


SlVector3 Tracer::shade(const HitRecord &hr) const {
    if (color) return hr.f.color;

    SlVector3 color(0.0);
    HitRecord dummy;

    constexpr auto epsilon = 0.00001;

    for (unsigned int i=0; i<lights.size(); i++) {
        const Light &light = lights[i];
        bool shadow = false;

        auto lightDirection = light.p - hr.p;
        normalize(lightDirection);

        Ray shadowRay(hr.p, lightDirection);

        // This part assumes that we are always far away from the objects
        for(auto& [geometry, f] : surfaces)
        {
            if(geometry->intersect(shadowRay, epsilon, std::numeric_limits<double>::max(), dummy))
            {
                if(l2Norm(hr.p - eye) > dummy.t)
                {
                    shadow = true;
                    break;
                }
            }
        }

        if (!shadow) {
            
            auto inVec = light.p - hr.p;
            normalize(inVec);

            auto& viewVec = hr.v;

            // Calculate reflection vector
            auto reflectVec = -inVec + 2 * dot(inVec, hr.n) * hr.n;
            normalize(reflectVec);

            // Calculate diffuse color
            auto diffuse = hr.f.kd * light.c * hr.f.color * std::max(dot(inVec, hr.n), 0.0);
            // Calculate specular color
            auto specular = hr.f.ks * light.c * std::pow(std::max(0.0, dot(reflectVec, viewVec)), hr.f.shine);
            color += diffuse + specular;
        }
    }

    // Step 4 Add code for computing reflection color here

    

    // Step 5 Add code for computing refraction color here

    return color;
}

SlVector3 Tracer::trace(const Ray &r, double t0, double t1) const {
    HitRecord hr;
    SlVector3 color(bcolor);
  
    bool hit = false;
    double minHitDistance = std::numeric_limits<double>::max();

    HitRecord temp;

    // Search for all objects for ray hits
    for(auto& [geometry, f] : surfaces)
    {
        if(geometry->intersect(r, t0, t1, temp))
        {
            if(temp.t < minHitDistance)
            {
                minHitDistance = temp.t;
                hr = temp;
                hr.f = f;
                hit = true;
            }
        }
    }

    if (hit) color = shade(hr);
    return color;
}


void Tracer::traceImage() {
    // set up coordinate system
    SlVector3 w = eye - at;
    w /= mag(w);
    SlVector3 u = cross(up,w);
    normalize(u);
    SlVector3 v = cross(w,u);
    normalize(v);

    double d = mag(eye - at);
    double h = tan((M_PI/180.0) * (angle/2.0)) * d;
    double l = -h;
    double r = h;
    double b = h;
    double t = -h;

    SlVector3 *pixel = im;

    for (unsigned int j=0; j<res[1]; j++) {
        for (unsigned int i=0; i<res[0]; i++, pixel++) {

            SlVector3 result(0.0,0.0,0.0);

            for (int k = 0; k < samples; k++) {

                double rx = 1.1 * rand() / RAND_MAX;
                double ry = 1.1 * rand() / RAND_MAX;

                double x = l + (r-l)*(i+rx)/res[0];
                double y = b + (t-b)*(j+ry)/res[1];
                SlVector3 dir = -d * w + x * u + y * v;
	
                Ray r(eye, dir);
                normalize(r.d);

                result += trace(r, hither, MAX);

            }
            (*pixel) = result / samples;
        }
    }
}

void Tracer::writeImage(const std::string &fname) {
#ifdef __APPLE__
    std::ofstream out(fname, std::ios::out | std::ios::binary);
#else
    std::ofstream out(fname.c_str(), std::ios_base::binary);
#endif
    out<<"P6"<<"\n"<<res[0]<<" "<<res[1]<<"\n"<<255<<"\n";
    SlVector3 *pixel = im;
    char val;
    for (unsigned int i=0; i<res[0]*res[1]; i++, pixel++) {
        val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[0])) * 255.0);
        out.write (&val, sizeof(unsigned char));
        val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[1])) * 255.0);
        out.write (&val, sizeof(unsigned char));
        val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[2])) * 255.0);
        out.write (&val, sizeof(unsigned char));
    }
    out.close();
}


int main(int argc, char *argv[]) {
    int c;
    double aperture = 0.0;
    int samples = 1;
    int maxraydepth = 5;
    bool color = false;
    while ((c = getopt(argc, argv, "a:s:d:c")) != -1) {
        switch(c) {
            case 'a':
            aperture = atof(optarg);
            break;
            case 's':
            samples = atoi(optarg);
            break;
            case 'c':
            color = true;
            break;
            case 'd':
            maxraydepth = atoi(optarg);
            break;
            default:
            abort();
        }
    }

    if (argc-optind != 2) {
        std::cout<<"usage: trace [opts] input.nff output.ppm"<<std::endl;
        for (unsigned int i=0; i<argc; i++) std::cout<<argv[i]<<std::endl;
        exit(0);
    }	

    Tracer tracer(argv[optind++]);
    tracer.aperture = aperture;
    tracer.samples = samples;
    tracer.color = color;
    tracer.maxraydepth = maxraydepth;
    tracer.traceImage();
    tracer.writeImage(argv[optind++]);
};
