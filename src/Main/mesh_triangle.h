#ifndef MESH_TRIANGLE_H
#define MESH_TRIANGLE_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

#include "rtweekend.h"

#include "hittable.h"
#include "hittable_list.h"
#include "OBJ_Loader.hpp"


class triangle : public hittable {
public:
    triangle() {}

    triangle(point3 v0, point3 v1, point3 v2, shared_ptr<material> m)
            : v0(v0), v1(v1), v2(v2), mat_ptr(m){
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(cross(e1, e2));
        area = cross(e1, e2).length() / 2;
    }

    triangle(point3 v0, point3 v1, point3 v2, point3 n0, point3 n1, point3 n2, vec3 t0, vec3 t1, vec3 t2, shared_ptr<material> m)
            : v0(v0), v1(v1), v2(v2), n0(n0), n1(n1), n2(n2), t0(t0), t1(t1), t2(t2), mat_ptr(m){
        e1 = v1 - v0;
        e2 = v2 - v0;
        has_normal = true;
        normal = normalize(cross(e1, e2));
        area = cross(e1, e2).length() / 2;
    }

    virtual bool hit(
            const ray& r, double t_min, double t_max, hit_record& rec) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

public:
    point3 v0;
    point3 v1;
    point3 v2;
    point3 e1;
    point3 e2;
    point3 n0, n1, n2;
    point3 t0, t1, t2;
    point3 normal;
    double area;
    shared_ptr<material> mat_ptr;
    bool has_normal;
};

bool triangle::hit(const ray &r, double t_min, double t_max, hit_record &rec) const {
    vec3 pvec = cross(r.dir, e2);
    double det = dot(e1, pvec);
    if (det == 0 || det < 0)
        return false;

    vec3 tvec = r.orig - v0;
    double u,v;
    u = dot(tvec, pvec);
    if (u < 0 || u > det)
        return false;

    vec3 qvec = cross(tvec, e1);
    v = dot(r.dir, qvec);
    if (v < 0 || u + v > det)
        return false;

    double invDet = 1 / det;

    double tnear = dot(e2, qvec) * invDet;
    if(tnear <= t_min || tnear >= t_max) return false;
    u *= invDet;
    v *= invDet;
    if(has_normal){
        rec.normal = (1 - u - v) * n0 + u * n1 + v * n2;
        vec3 text_p = (1 - u - v) * t0 + u * t1 + v * t2;
        rec.u = text_p.x();
        rec.v = text_p.y();
    }else{
        rec.normal = normal;
        rec.u = u;
        rec.v = v;
    }
    rec.t = tnear;
    rec.p = r.at(rec.t);
    vec3 outward_normal = rec.normal;
    rec.set_face_normal(r, outward_normal);
    rec.mat_ptr = mat_ptr;

    return true;
}

bool triangle::bounding_box(double time0, double time1, aabb& output_box) const {
    vec3 min, max;
    for(int i=0; i<3; i++){
        min.e[i] = std::min(std::min(v0.e[i], v1.e[i]), v2.e[i]);
        max.e[i] = std::max(std::max(v0.e[i], v1.e[i]), v2.e[i]);
    }
    output_box = aabb(
            min,
            max);
    return true;
}

shared_ptr<hittable> read_obj_model_triangle(const std::string& filename, shared_ptr<material> m, vec3 trans, vec3 rotation, vec3 scale){
    objl::Loader loader;
    hittable_list mesh_tri;
    loader.LoadFile(filename);
    std::vector<shared_ptr<hittable>> obj_list;

    // above !!;
    //assert(loader.LoadedMeshes.size() == 1);
    auto mesh = loader.LoadedMeshes[0];
    std::cout << "model size: " << mesh.Vertices.size() / 3 << std::endl;

    for (int i = 0; i < mesh.Vertices.size(); i += 3){
        // TODO: add trans & rotation
        mesh_tri.add(make_shared<triangle>(
                        point3(mesh.Vertices[i+0].Position.X * scale.x(), mesh.Vertices[i+0].Position.Y * scale.y(), mesh.Vertices[i+0].Position.Z* scale.z()),
                        point3(mesh.Vertices[i+1].Position.X * scale.x(), mesh.Vertices[i+1].Position.Y * scale.y(), mesh.Vertices[i+1].Position.Z* scale.z()),
                        point3(mesh.Vertices[i+2].Position.X * scale.x(), mesh.Vertices[i+2].Position.Y * scale.y(), mesh.Vertices[i+2].Position.Z* scale.z()),
                        point3(mesh.Vertices[i+0].Normal.X, mesh.Vertices[i+0].Normal.Y, mesh.Vertices[i+0].Normal.Z),
                        point3(mesh.Vertices[i+1].Normal.X, mesh.Vertices[i+1].Normal.Y, mesh.Vertices[i+1].Normal.Z),
                        point3(mesh.Vertices[i+2].Normal.X, mesh.Vertices[i+2].Normal.Y, mesh.Vertices[i+2].Normal.Z),
                        point3(mesh.Vertices[i+0].TextureCoordinate.X, mesh.Vertices[i+0].TextureCoordinate.Y, 0),
                        point3(mesh.Vertices[i+1].TextureCoordinate.X, mesh.Vertices[i+1].TextureCoordinate.Y, 0),
                        point3(mesh.Vertices[i+2].TextureCoordinate.X, mesh.Vertices[i+2].TextureCoordinate.Y, 0),
                           m));
    }

    return make_shared<translate>(
            make_shared<rotate_y>(
                    make_shared<bvh_node>(mesh_tri, 0.0, 1.0), rotation.y()),
            vec3(trans));

}

#endif
