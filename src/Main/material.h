#ifndef MATERIAL_H
#define MATERIAL_H
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
#include "texture.h"


class material {
    /******************************
        材质基类，包括发光和散射方法
    *******************************/
    public:
        // 用于光源物体
        virtual color emitted(double u, double v, const point3& p) const {
            return color(0,0,0);
        }

        // 用于对对象的吸收率和光线散射进行定义
        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
        ) const = 0;
};


class lambertian : public material {
    /******************************
        朗博材质，对于光照的吸收沿法线的cos(theta)衰减
    ******************************/
    public:
        lambertian(const color& a) : albedo(make_shared<solid_color>(a)) {}
        lambertian(shared_ptr<texture> a) : albedo(a) {}

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
        ) const override {
            // 光线散射方向为法线向量加一个球形的采样，模拟cos(theta)分布
            auto scatter_direction = rec.normal + random_unit_vector();

            // 处理采样到原点的点
            if (scatter_direction.near_zero())
                scatter_direction = rec.normal;

            scattered = ray(rec.p, scatter_direction, r_in.time());
            attenuation = albedo->value(rec.u, rec.v, rec.p);
            return true;
        }

    public:
        shared_ptr<texture> albedo;
};

class sky : public material {
    /******************************
        天空盒材质，
    ******************************/
    public:
        sky(shared_ptr<texture> a) : emit(a) {}
        sky(color c) : emit(make_shared<solid_color>(c)) {}

        virtual bool scatter(
                const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
        ) const override {
            return false;
        }

        virtual color emitted(double u, double v, const point3& p) const override {
            return emit->value(v, u, p);
        }

    public:
        shared_ptr<texture> emit;
};

class BRDF : public material {
    /******************************
        BRDF材质
    ******************************/
    public:
        BRDF(shared_ptr<texture> a) : BRDF_texture(a) {}

        virtual bool scatter(
                const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
                ) const override {
            // 沿法线所在半球随机采样
            auto scatter_direction = random_in_hemisphere(rec.normal);
            if (scatter_direction.near_zero())
                scatter_direction = rec.normal;
            
            // 求解入射光线与出射光线的中线
            vec3 half_dir = normalize(r_in.direction() + scatter_direction);

            // 求解法线同half dir的角度作为BRDF贴图的横坐标
            double u = dot(rec.normal, half_dir) * 0.5 + 0.5;

            // 求解入射光线与half dir的角度作为BRDF贴图的纵坐标
            double v = dot(normalize(r_in.direction()), half_dir);

            scattered = ray(rec.p, scatter_direction, r_in.time());
            attenuation = BRDF_texture->value(u, v, rec.p);
            return true;
        }

    public:
        shared_ptr<texture> BRDF_texture;
};


class metal : public material {
    /******************************
        金属材质
    ******************************/
    public:
        metal(const color& a, double f) : albedo(a), fuzz(f < 1 ? f : 1) {}

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
        ) const override {
            // 求解镜面反射光线方向
            vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);

            // 根据粗糙程度对反射光线进行随机偏转
            scattered = ray(rec.p, reflected + fuzz*random_in_unit_sphere(), r_in.time());
            attenuation = albedo;
            return (dot(scattered.direction(), rec.normal) > 0);
        }

    public:
        color albedo;
        double fuzz;
};


class dielectric : public material {
    /******************************
        非传导性介质材质
    ******************************/
    public:
        dielectric(double index_of_refraction) : ir(index_of_refraction) {}

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
        ) const override {
            // 透光率
            attenuation = color(1.0, 1.0, 1.0);

            // 折射率，从内到外时折射率取反
            double refraction_ratio = rec.front_face ? (1.0/ir) : ir;

            vec3 unit_direction = unit_vector(r_in.direction());
            double cos_theta = fmin(dot(-unit_direction, rec.normal), 1.0);
            double sin_theta = sqrt(1.0 - cos_theta*cos_theta);

            // 计算能否实现折射
            bool cannot_refract = refraction_ratio * sin_theta > 1.0;
            vec3 direction;

            // 如果不能折射或者根据反射与折射比值进行抽样模拟模拟为反射时反射，否则折射
            if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double())
                direction = reflect(unit_direction, rec.normal);
            else
                direction = refract(unit_direction, rec.normal, refraction_ratio);

            scattered = ray(rec.p, direction, r_in.time());
            return true;
        }

    public:
        double ir; // Index of Refraction

    private:
        static double reflectance(double cosine, double ref_idx) {
            // Use Schlick's approximation for reflectance.
            auto r0 = (1-ref_idx) / (1+ref_idx);
            r0 = r0*r0;
            return r0 + (1-r0)*pow((1 - cosine),5);
        }
};


class diffuse_light : public material {
    /******************************
        光照材质
    ******************************/
    public:
        diffuse_light(shared_ptr<texture> a) : emit(a) {}
        diffuse_light(color c) : emit(make_shared<solid_color>(c)) {}

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
        ) const override {
            return false;
        }

        virtual color emitted(double u, double v, const point3& p) const override {
            // 命中后返回贴图值
            return emit->value(u, v, p);
        }

    public:
        shared_ptr<texture> emit;
};


class isotropic : public material {
    /******************************
        各项同性的
    ******************************/
    public:
        isotropic(color c) : albedo(make_shared<solid_color>(c)) {}
        isotropic(shared_ptr<texture> a) : albedo(a) {}

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
        ) const override {
            scattered = ray(rec.p, random_in_unit_sphere(), r_in.time());
            attenuation = albedo->value(rec.u, rec.v, rec.p);
            return true;
        }

    public:
        shared_ptr<texture> albedo;
};

#endif
