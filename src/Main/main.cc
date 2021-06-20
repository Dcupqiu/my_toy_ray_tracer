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

#include "box.h"
#include "bvh.h"
#include "camera.h"
#include "color.h"
#include "constant_medium.h"
#include "hittable_list.h"
#include "material.h"
#include "moving_sphere.h"
#include "sphere.h"
#include "texture.h"
#include "mesh_triangle.h"
#include "skybox.h"
#include <omp.h>
#include "opencv4/opencv2/opencv.hpp"
#include <boost/timer.hpp>

#include <iostream>
#include <unistd.h>

sky_box make_sky_box() {
    /***************
    生成天空盒
    ***************/

    // 读取天空盒6面
    auto front = make_shared<sky>(make_shared<image_texture>("../models/skybox/front.png"));
    auto back = make_shared<sky>(make_shared<image_texture>("../models/skybox/back.png"));
    auto left = make_shared<sky>(make_shared<image_texture>("../models/skybox/left.png"));
    auto right = make_shared<sky>(make_shared<image_texture>("../models/skybox/right.png"));
    auto top = make_shared<sky>(make_shared<image_texture>("../models/skybox/top.png"));
    auto bottom = make_shared<sky>(make_shared<image_texture>("../models/skybox/bottom.png"));

    // 返回天空盒
    return sky_box({back, front, top, bottom, right, left});
}

color ray_color(const ray &r, const color &background, const hittable &world, int depth) {
    hit_record rec;

    // 如果达到了最大碰撞深度，不再进行碰撞
    if (depth <= 0)
        return color(0, 0, 0);

    // 如果光线啥都没碰到，从背景中取颜色
    if (!world.hit(r, 0.001, infinity, rec))
        return background;

    ray scattered;
    color attenuation;
    color emitted = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);

    // 如果碰撞到的物体不会再进行散射，则直接返回其光照值
    if (!rec.mat_ptr->scatter(r, rec, attenuation, scattered))
        return emitted;

    // 返回照度与后续照度的叠加
    return emitted + attenuation * ray_color(scattered, background, world, depth - 1);
}

color ray_color_sky_box(const ray &r, const hittable &sky_box, const hittable &world, int depth) {
    hit_record rec;

    // 如果达到了最大碰撞深度，不再进行碰撞
    if (depth <= 0)
        return color(0, 0, 0);

    // 如果光线啥都没碰到，从天空盒中取颜色
    if (!world.hit(r, 0.001, infinity, rec)) {
        ray r_t(r);
        r_t.orig = {0, 0, 0};
        sky_box.hit(r_t, 0.001, infinity, rec);
        auto back = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);
        return back;
    }

    ray scattered;
    color attenuation;
    color emitted = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);

    // 如果碰撞到的物体不会再进行散射，则直接返回其光照值
    if (!rec.mat_ptr->scatter(r, rec, attenuation, scattered))
        return emitted;

    // 返回照度与后续照度的叠加
    return emitted + attenuation * ray_color_sky_box(scattered, sky_box, world, depth - 1);
}


hittable_list my_scene1() {
    hittable_list objects;

    // 创建贴图材质
    auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
    auto material = make_shared<BRDF>(perlin_texture);
    auto metal_m = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);

    // 创建双层玻璃球
    objects.add(make_shared<sphere>(vec3(3, 1, 3), 0.8, make_shared<dielectric>(1.5)));
    objects.add(make_shared<inner_sphere>(vec3(3, 1, 3), 0.6, make_shared<dielectric>(1.5)));


    // 创建透明兔子
    objects.add(
            read_obj_model_triangle("../models/bunny4.obj", make_shared<dielectric>(1.5), vec3(4, 0, 0), vec3(0, 0, 0),
                                    vec3(0.4, 0.4, 0.4)));
    // 创建地板
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<metal>(color(0.6, 0.6, 0.6), 0.)));

    // 创建龙
    objects.add(read_obj_model_triangle("../models/dragon2.obj", material, vec3(-0.5, 0, -3), vec3(0, 80, 0),
                                        vec3(0.5, 0.5, 0.5)));

    // 创建牛
    objects.add(read_obj_model_triangle("../models/spot_triangulated_good.obj", make_shared<lambertian>(spot_texture),
                                        vec3(0, 1, 5), vec3(0, -60, 0), vec3(1.5, 1.5, 1.5)));
    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list my_scene2() {
    hittable_list objects;

    auto checker_tex = make_shared<checker_texture>(
            color(0.75, 0.1, 0.3),
            color(0.9, 0.9, 0.9));

    auto metal_tex = make_shared<metal>(color(0.8, 0.8, 0.9), 0.0);
    auto perlin_mat = make_shared<perlin_brdf_texture>(4);
    auto perlin_tex = make_shared<BRDF>(perlin_mat);
    auto colorspot_tex = make_shared<image_texture>("../models/spot_texture.png");
    auto die_tex = make_shared<dielectric>(1.5);

    //floor
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<lambertian>(checker_tex)));
    //objects
    objects.add(read_obj_model_triangle(
            "../models/Rabbit.obj",
            metal_tex,
            vec3(1.5, 0, 2.5),
            vec3(0, 120, 0),
            vec3(1, 1, 1)));

    objects.add(read_obj_model_triangle(
            "../models/Dog2.obj",
            perlin_tex,
            vec3(1, 0, 0.5),
            vec3(0, 60, 0),
            vec3(0.2, 0.2, 0.2)));

    objects.add(read_obj_model_triangle(
            "../models/spot_triangulated_good.obj",
            make_shared<lambertian>(colorspot_tex),
            vec3(2, 1, -2.5),
            vec3(0, -120, 0),
            vec3(1.2, 1.2, 1.2)));

    objects.add(read_obj_model_triangle(
            "../models/SeaUrchin2.obj",
            die_tex,
            vec3(5, 0, 0),
            vec3(0, 30, 0),
            vec3(0.08, 0.08, 0.08)));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list my_scene3() {
    hittable_list objects;

    auto checker_tex = make_shared<checker_texture>(
            color(0.75, 0.1, 0.3),
            color(0.9, 0.9, 0.9));
    auto noise_tex = make_shared<noise_texture>(2);

    auto metal_tex = make_shared<metal>(color(0.8, 0.8, 0.9), 0.0);
    auto perlin_mat = make_shared<perlin_brdf_texture>(4);
    auto perlin_tex = make_shared<BRDF>(perlin_mat);
    auto colorspot_tex = make_shared<image_texture>("../models/spot_texture.png");
    auto die_tex = make_shared<dielectric>(1.5);

    //floor
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<lambertian>(noise_tex)));
    //objects
    objects.add(read_obj_model_triangle(
            "../models/Rabbit.obj",
            metal_tex,
            vec3(1.5, 0, 2.5),
            vec3(0, 120, 0),
            vec3(1, 1, 1)));

    objects.add(read_obj_model_triangle(
            "../models/Dog2.obj",
            perlin_tex,
            vec3(1, 0, 0.5),
            vec3(0, 60, 0),
            vec3(0.2, 0.2, 0.2)));

    objects.add(read_obj_model_triangle(
            "../models/spot_triangulated_good.obj",
            make_shared<lambertian>(colorspot_tex),
            vec3(2, 1, -2.5),
            vec3(0, -120, 0),
            vec3(1.2, 1.2, 1.2)));

    objects.add(read_obj_model_triangle(
            "../models/SeaUrchin2.obj",
            die_tex,
            vec3(5, 0, 0),
            vec3(0, 30, 0),
            vec3(0.08, 0.08, 0.08)));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list cornell_box2() {
    hittable_list objects;

    auto purple = make_shared<lambertian>(color(0.54, 0.25, 0.46));
    auto yellow = make_shared<lambertian>(color(0.98, 0.65, 0.20));
    auto white = make_shared<lambertian>(color(0.73, 0.73, 0.73));
    auto blue = make_shared<lambertian>(color(0.09, 0.38, 0.67));
    auto gray = make_shared<lambertian>(color(0.57, 0.50, 0.45));
    auto light = make_shared<diffuse_light>(color(7, 7, 7));
    auto red = color(0.67, 0.22, 0.18);
    auto green = color(0.14, 0.50, 0.40);

    auto metal_tex = make_shared<metal>(red, 1.0);
    auto metal_tex2 = make_shared<metal>(green, 0.2);
    auto perlin_mat = make_shared<perlin_brdf_texture>(4);
    auto perlin_tex = make_shared<BRDF>(perlin_mat);
    auto colorspot_tex = make_shared<image_texture>("../models/spot_texture.png");
    auto die_tex = make_shared<dielectric>(1.5);

    //make a room
    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, purple));
    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, yellow));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, blue));
    objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));
    objects.add(make_shared<xz_rect>(100, 456, 114, 446, 554, light));

    //objects

    objects.add(make_shared<sphere>(vec3(180, 300, 300), 90, make_shared<dielectric>(1.5)));
    objects.add(make_shared<inner_sphere>(vec3(180, 300, 300), 80, make_shared<dielectric>(1.5)));

    //box
    shared_ptr<hittable> box_trian = make_shared<box>(point3(0, 0, 0), point3(165, 330, 165), gray);
    box_trian = make_shared<rotate_y>(box_trian, 20);
    box_trian = make_shared<translate>(box_trian, vec3(265, 0, 295));
    objects.add(box_trian);

    shared_ptr<hittable> box_square = make_shared<box>(point3(0, 0, 0), point3(165, 165, 165), white);
    box_square = make_shared<rotate_y>(box_square, -75);
    box_square = make_shared<translate>(box_square, vec3(130, 0, 65));
    objects.add(box_square);
    //others
    objects.add(read_obj_model_triangle(
            "../models/SeaUrchin2.obj",
            metal_tex2,
            vec3(380, 330, 340),
            vec3(0, 0, 0),
            vec3(7, 7, 7)));
    objects.add(read_obj_model_triangle(
            "../models/Rabbit.obj",
            metal_tex,
            vec3(30, 170, 210),
            vec3(0, 120, 0),
            vec3(30, 30, 30)));
    objects.add(read_obj_model_triangle(
            "../models/Dog2.obj",
            perlin_tex,
            vec3(320, 0, 210),
            vec3(0, -150, 0),
            vec3(15, 15, 15)));
    objects.add(read_obj_model_triangle(
            "../models/spot_triangulated_good.obj",
            make_shared<lambertian>(colorspot_tex),
            vec3(335, 50, 300),
            vec3(0, -120, 0),
            vec3(90, 90, 90)));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list lambertian_scene() {
    hittable_list objects;

    //auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto lambertian_tex = make_shared<lambertian>(spot_texture);
    auto red = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto noise_tex = make_shared<noise_texture>(4);
    auto noise = make_shared<lambertian>(noise_tex);
//    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
//    auto material = make_shared<BRDF>(perlin_texture);
//    auto metal_m = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);
    // Floor
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<metal>(color(0.6, 0.6, 0.6), 0.)));
    objects.add(make_shared<sphere>(vec3(0, 1, 6), 1, lambertian_tex));
    objects.add(make_shared<sphere>(vec3(0, 1, 3), 1, red));
    objects.add(make_shared<sphere>(vec3(0, 1, 0), 1, white));
    objects.add(make_shared<sphere>(vec3(0, 1, -3), 1, green));
    objects.add(make_shared<sphere>(vec3(0, 1, -6), 1, noise));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list perlin_scene() {
    hittable_list objects;

    //auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto lambertian_tex = make_shared<lambertian>(spot_texture);
    auto red = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto noise_tex = make_shared<noise_texture>(4);
    auto noise = make_shared<lambertian>(noise_tex);
//    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
//    auto material = make_shared<BRDF>(perlin_texture);
//    auto metal_m = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);
    // Floor
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<metal>(color(0.6, 0.6, 0.6), 0.)));
    objects.add(make_shared<sphere>(vec3(0, 1, 6), 1, make_shared<lambertian>(make_shared<noise_texture>(1))));
    objects.add(make_shared<sphere>(vec3(0, 1, 3), 1, make_shared<lambertian>(make_shared<noise_texture>(2))));
    objects.add(make_shared<sphere>(vec3(0, 1, 0), 1, make_shared<lambertian>(make_shared<noise_texture>(4))));
    objects.add(make_shared<sphere>(vec3(0, 1, -3), 1, make_shared<lambertian>(make_shared<noise_texture>(6))));
    objects.add(make_shared<sphere>(vec3(0, 1, -6), 1, make_shared<lambertian>(make_shared<noise_texture>(8))));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list dielectric_scene() {
    hittable_list objects;

    auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
    auto material = make_shared<BRDF>(perlin_texture);
    auto metal_m = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);
    // Floor
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<metal>(color(0.6, 0.6, 0.6), 0.)));
    objects.add(make_shared<sphere>(vec3(0, 1, 6), 1, make_shared<dielectric>(1.8)));
    objects.add(make_shared<sphere>(vec3(0, 1, 3), 1, make_shared<dielectric>(1.6)));
    objects.add(make_shared<sphere>(vec3(0, 1, 0), 1, make_shared<dielectric>(1.4)));
    objects.add(make_shared<sphere>(vec3(0, 1, -3), 1, make_shared<dielectric>(1.2)));
    objects.add(make_shared<sphere>(vec3(0, 1, -6), 1, make_shared<dielectric>(1.5)));
    objects.add(make_shared<inner_sphere>(vec3(0, 1, -6), 0.7, make_shared<dielectric>(1.5)));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list metal_scene() {
    hittable_list objects;

    //auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto lambertian_tex = make_shared<lambertian>(spot_texture);
    auto noise_tex = make_shared<noise_texture>(4);
//    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
//    auto material = make_shared<BRDF>(perlin_texture);
    auto metal1 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.);
    auto metal2 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.1);
    auto metal3 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);
    auto metal4 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.4);
    auto metal5 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.8);
    // Floor
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<metal>(color(0.6, 0.6, 0.6), 0.)));
    objects.add(make_shared<sphere>(vec3(0, 1, 6), 1, metal1));
    objects.add(make_shared<sphere>(vec3(0, 1, 3), 1, metal2));
    objects.add(make_shared<sphere>(vec3(0, 1, 0), 1, metal3));
    objects.add(make_shared<sphere>(vec3(0, 1, -3), 1, metal4));
    objects.add(make_shared<sphere>(vec3(0, 1, -6), 1, metal5));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list BRDF_scene() {
    hittable_list objects;

    //auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto lambertian_tex = make_shared<lambertian>(spot_texture);
    auto noise_tex = make_shared<noise_texture>(4);
//    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
    auto material1 = make_shared<BRDF>(make_shared<perlin_brdf_texture>(1));
    auto material2 = make_shared<BRDF>(make_shared<perlin_brdf_texture>(2));
    auto material3 = make_shared<BRDF>(make_shared<perlin_brdf_texture>(4));
    auto material4 = make_shared<BRDF>(make_shared<perlin_brdf_texture>(6));
    auto material5 = make_shared<BRDF>(make_shared<perlin_brdf_texture>(8));
//    auto metal1 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.);
//    auto metal2 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.1);
//    auto metal3 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);
//    auto metal4 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.4);
//    auto metal5 = make_shared<metal>(color(0.8, 0.8, 0.9), 0.8);
    // Floor
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<metal>(color(0.6, 0.6, 0.6), 0.)));
    objects.add(make_shared<sphere>(vec3(0, 1, 6), 1, material1));
    objects.add(make_shared<sphere>(vec3(0, 1, 3), 1, material2));
    objects.add(make_shared<sphere>(vec3(0, 1, 0), 1, material3));
    objects.add(make_shared<sphere>(vec3(0, 1, -3), 1, material4));
    objects.add(make_shared<sphere>(vec3(0, 1, -6), 1, material5));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list bvh_test() {
    hittable_list objects;

    auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
    auto material = make_shared<BRDF>(perlin_texture);
    auto metal_m = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);

    objects.add(read_obj_model_triangle("../models/dragon2.obj", material, vec3(-0.5, 0, -3), vec3(0, 80, 0),
                                        vec3(0.5, 0.5, 0.5)));
    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

hittable_list no_bvh_test() {
    hittable_list objects;

    auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
    auto material = make_shared<BRDF>(perlin_texture);
    auto metal_m = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);

    objects.add(read_obj_model_triangle_no_bvh("../models/dragon2.obj", material, vec3(-0.5, 0, -3), vec3(0, 80, 0),
                                        vec3(0.5, 0.5, 0.5)));
    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}

void parse_arg(int argc, char *argv[], int &spp, int &scene) {
    int opt;
    while ((opt = getopt(argc, argv, "hs:p:")) != -1) {
        switch (opt) {
            case 'h':
                printf("Usage: %s [-s scene] [-p spp]\n", argv[0]);
                exit(0);
                break;
            case 's':
                scene = atoi(optarg);
                break;
            case 'p':
                spp = atoi(optarg);
                break;
            default:
                break;
        }
    }
}

int main(int argc, char *argv[]) {

    auto aspect_ratio = 16.0 / 9.0; // 图像比例
    int image_width = 960; // 图像宽度
    int samples_per_pixel = 1; // 每像素采样数
    int max_depth = 50; // 最大碰撞深度

    // World
    hittable_list world; // 碰撞体的集合——世界

    int scene = 10; // Scene_id

    point3 lookfrom; // 视点原点
    point3 lookat; // 视点方向
    auto vfov = 40.0; // 视角
    auto aperture = 0.0; // 光圈
    color background(0, 0, 0); // 背景颜色
    auto sky_box = make_sky_box(); // 天空盒实现
    bool using_sky_box = false;


    // 选择对应的场景进行渲染
    parse_arg(argc, argv, samples_per_pixel, scene);
    printf("Samples Per Pixel : %d\nScene : %d\n", samples_per_pixel, scene);
    switch (scene) {
        case 1:
            world = my_scene1();
            using_sky_box = true;
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            vfov = 75.0;
            max_depth = 25;
            break;

        case 2:
            world = my_scene2();
            using_sky_box = true;
            background = color(0.70, 0.80, 1.00);
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            max_depth = 25;
            vfov = 75.0;
            break;

        case 3:
            world = cornell_box2();
            aspect_ratio = 1.0;
            image_width = 600;
            lookfrom = point3(278, 278, -800);
            lookat = point3(278, 278, 0);
            max_depth = 25;
            vfov = 40.0;
            break;

        case 4:
            world = my_scene3();
            using_sky_box = true;
            background = color(0.70, 0.80, 1.00);
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            max_depth = 25;
            vfov = 75.0;
            break;

        case 5:
            world = lambertian_scene();
            using_sky_box = true;
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            vfov = 60.0;
            max_depth = 25;
            break;

        default:
        case 6:
            world = dielectric_scene();
            using_sky_box = true;
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            vfov = 60.0;
            max_depth = 25;
            break;

        case 7:
            world = metal_scene();
            using_sky_box = true;
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            vfov = 60.0;
            max_depth = 25;
            break;

        case 8:
            world = BRDF_scene();
            using_sky_box = true;
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            vfov = 60.0;
            max_depth = 25;
            break;

        case 9:
            world = perlin_scene();
            using_sky_box = true;
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            vfov = 60.0;
            max_depth = 25;
            break;

        case 10:
            world = bvh_test();
            using_sky_box = true;
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            vfov = 60.0;
            max_depth = 25;
            break;

        case 11:
            world = no_bvh_test();
            using_sky_box = true;
            lookfrom = point3(7, 3, 0);
            lookat = point3(0, 0, 0);
            vfov = 60.0;
            max_depth = 25;
            break;
    }

    // 相机
    const vec3 vup(0, 1, 0); // 相机正向
    const auto dist_to_focus = 10.0; // 焦距
    const int image_height = static_cast<int>(image_width / aspect_ratio); // 渲染图像高度

    camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, 0.0, 1.0); // 生成相机对象，以实现光线生成

    // 渲染
    std::vector<color> framebuffer(image_width * image_height); // 渲染的buffer，以供并行渲染
    boost::timer t_ogm;
    if (using_sky_box) {
#pragma omp parallel for collapse(2) schedule(dynamic, 8) num_threads(6)
        for (int j = image_height - 1; j >= 0; j--) {
            for (int i = 0; i < image_width; ++i) {
                color pixel_color(0, 0, 0);
                for (int s = 0; s < samples_per_pixel; ++s) {
                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = cam.get_ray(u, v);
                    //                pixel_color += ray_color(r, background, world, max_depth); // Background 渲染
                    pixel_color += ray_color_sky_box(r, sky_box, world, max_depth); // 天空盒渲染
                }
                framebuffer[(image_height - j - 1) * image_width + i] = pixel_color;
            }
        }
    } else {
#pragma omp parallel for collapse(2) schedule(dynamic, 8) num_threads(6)
        for (int j = image_height - 1; j >= 0; j--) {
            for (int i = 0; i < image_width; ++i) {
                color pixel_color(0, 0, 0);
                for (int s = 0; s < samples_per_pixel; ++s) {
                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = cam.get_ray(u, v);
                    pixel_color += ray_color(r, background, world, max_depth); // Background 渲染
//                    pixel_color += ray_color_sky_box(r, sky_box, world, max_depth); // 天空盒渲染
                }
                framebuffer[(image_height - j - 1) * image_width + i] = pixel_color;
            }
        }
    }
    float time_cost = t_ogm.elapsed();
    std::cout << "Time_cost: " << time_cost << std::endl;
    // 渲染结束

    // 从Buffer转为图像显示并保存
    int h = image_height;
    int w = image_width;
    cv::Mat image(h, w, CV_8UC3);
    cv::Mat image2(h, w, CV_8UC3);
    for (auto i = 0; i < image_height * image_width; ++i) {
        auto r = framebuffer[i].x();
        auto g = framebuffer[i].y();
        auto b = framebuffer[i].z();

        if (r != r) r = 0.0;
        if (g != g) g = 0.0;
        if (b != b) b = 0.0;

        auto scale = 1.0 / samples_per_pixel;
        r = sqrt(scale * r);
        g = sqrt(scale * g);
        b = sqrt(scale * b);


        static unsigned char color[3];
        color[0] = (unsigned char) (static_cast<int>(256 * clamp(r, 0.0, 0.999)));
        color[1] = (unsigned char) (static_cast<int>(256 * clamp(g, 0.0, 0.999)));
        color[2] = (unsigned char) (static_cast<int>(256 * clamp(b, 0.0, 0.999)));
        image.at<cv::Vec3b>(i / image_width, i % image_width)[0] = color[2];
        image.at<cv::Vec3b>(i / image_width, i % image_width)[1] = color[1];
        image.at<cv::Vec3b>(i / image_width, i % image_width)[2] = color[0];
    }
    cv::imwrite("./scene.jpg", image);
    cv::imshow("test", image);
    cv::waitKey();
    // 结束
    std::cout << "Image width: " << image_width << std::endl;
    std::cout << "Image height: " << image_height << std::endl;
    std::cout << "Samples per pixel: " << samples_per_pixel << std::endl;
}
