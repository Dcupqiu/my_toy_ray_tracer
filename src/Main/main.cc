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

sky_box make_sky_box(){
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

color ray_color(const ray& r, const color& background, const hittable& world, int depth) {
    hit_record rec;

    // 如果达到了最大碰撞深度，不再进行碰撞
    if (depth <= 0)
        return color(0,0,0);

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
    return emitted + attenuation * ray_color(scattered, background, world, depth-1);
}

color ray_color_sky_box(const ray& r, const hittable& sky_box, const hittable& world, int depth) {
    hit_record rec;

    // 如果达到了最大碰撞深度，不再进行碰撞
    if (depth <= 0)
        return color(0,0,0);

    // 如果光线啥都没碰到，从背景中取颜色
    if (!world.hit(r, 0.001, infinity, rec)){
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
    return emitted + attenuation * ray_color_sky_box(scattered, sky_box, world, depth-1);
}


hittable_list random_scene() {
    hittable_list world;

    auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));

    world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(checker)));

    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            auto choose_mat = random_double();
            point3 center(a + 0.9*random_double(), 0.2, b + 0.9*random_double());

            if ((center - vec3(4, 0.2, 0)).length() > 0.9) {
                shared_ptr<material> sphere_material;

                if (choose_mat < 0.8) {
                    // diffuse
                    auto albedo = color::random() * color::random();
                    sphere_material = make_shared<lambertian>(albedo);
                    auto center2 = center + vec3(0, random_double(0,.5), 0);
                    world.add(make_shared<moving_sphere>(
                        center, center2, 0.0, 1.0, 0.2, sphere_material));
                } else if (choose_mat < 0.95) {
                    // metal
                    auto albedo = color::random(0.5, 1);
                    auto fuzz = random_double(0, 0.5);
                    sphere_material = make_shared<metal>(albedo, fuzz);
                    world.add(make_shared<sphere>(center, 0.2, sphere_material));
                } else {
                    // glass
                    sphere_material = make_shared<dielectric>(1.5);
                    world.add(make_shared<sphere>(center, 0.2, sphere_material));
                }
            }
        }
    }

    auto material1 = make_shared<dielectric>(1.5);
    world.add(make_shared<sphere>(point3(0, 1, 0), 1.0, material1));

    auto material2 = make_shared<lambertian>(color(0.4, 0.2, 0.1));
    world.add(make_shared<sphere>(point3(-4, 1, 0), 1.0, material2));

    auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
    world.add(make_shared<sphere>(point3(4, 1, 0), 1.0, material3));

    return hittable_list(make_shared<bvh_node>(world, 0.0, 1.0));
}


hittable_list my_scene1() {
    hittable_list objects;

    auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    auto spot_texture = make_shared<image_texture>("../models/spot_texture.png");
    auto perlin_texture = make_shared<perlin_brdf_texture>(4);
    auto material = make_shared<BRDF>(perlin_texture);
    auto metal_m = make_shared<metal>(color(0.8, 0.8, 0.9), 0.2);
    objects.add(make_shared<sphere>(vec3(3, 1, 3), 0.8, make_shared<dielectric>(1.5)));
    objects.add(make_shared<inner_sphere>(vec3(3, 1, 3), 0.6, make_shared<dielectric>(1.5)));


    objects.add(read_obj_model_triangle("../models/bunny4.obj", make_shared<dielectric>(1.5), vec3(4, 0, 0), vec3(0, 0, 0), vec3(0.4, 0.4, 0.4)));
    objects.add(make_shared<xz_rect>(-30, 30, -30, 30, 0, make_shared<metal>(color(0.6, 0.6, 0.6), 0.)));
    objects.add(read_obj_model_triangle("../models/dragon2.obj", material, vec3(-0.5, 0, -3), vec3(0, 80, 0), vec3(0.5, 0.5, 0.5)));
    objects.add(read_obj_model_triangle("../models/spot_triangulated_good.obj", make_shared<lambertian>(spot_texture), vec3(0, 1, 5), vec3(0, -60, 0), vec3(1.5, 1.5, 1.5)));
    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}


hittable_list two_perlin_spheres() {
    hittable_list objects;

    auto pertext = make_shared<noise_texture>(4);
    objects.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(pertext)));
    objects.add(make_shared<sphere>(point3(0,2,0), 2, make_shared<lambertian>(pertext)));

    return objects;
}


hittable_list earth() {
    auto earth_texture = make_shared<image_texture>("earthmap.jpg");
    auto earth_surface = make_shared<lambertian>(earth_texture);
    auto globe = make_shared<sphere>(point3(0,0,0), 2, earth_surface);

    return hittable_list(globe);
}


hittable_list simple_light() {
    hittable_list objects;

    auto pertext = make_shared<noise_texture>(4);
    objects.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(pertext)));
    objects.add(make_shared<sphere>(point3(0,2,0), 2, make_shared<lambertian>(pertext)));

    auto difflight = make_shared<diffuse_light>(color(4,4,4));
    objects.add(make_shared<sphere>(point3(0,7,0), 2, difflight));
    objects.add(make_shared<xy_rect>(3, 5, 1, 3, -2, difflight));

    return objects;
}


hittable_list cornell_box() {
    hittable_list objects;

    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));

    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
    objects.add(make_shared<xz_rect>(213, 343, 227, 332, 554, light));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, white));
    objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));

    shared_ptr<hittable> box1 = make_shared<box>(point3(0,0,0), point3(165,330,165), white);
    box1 = make_shared<rotate_y>(box1, 15);
    box1 = make_shared<translate>(box1, vec3(265,0,295));
    objects.add(box1);

    shared_ptr<hittable> box2 = make_shared<box>(point3(0,0,0), point3(165,165,165), white);
    box2 = make_shared<rotate_y>(box2, -18);
    box2 = make_shared<translate>(box2, vec3(130,0,65));
    objects.add(box2);

    objects.add(read_obj_model_triangle("../models/bunny2.obj", make_shared<metal>(color(0.8, 0.8, 0.9), 1.0), vec3(380, 300, 340), vec3(0, 0, 0), vec3(20, 20, 20)));

    return hittable_list(make_shared<bvh_node>(objects, 0.0, 1.0));
}


hittable_list cornell_smoke() {
    hittable_list objects;

    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto light = make_shared<diffuse_light>(color(7, 7, 7));

    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
    objects.add(make_shared<xz_rect>(113, 443, 127, 432, 554, light));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, white));
    objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));

    shared_ptr<hittable> box1 = make_shared<box>(point3(0,0,0), point3(165,330,165), white);
    box1 = make_shared<rotate_y>(box1, 15);
    box1 = make_shared<translate>(box1, vec3(265,0,295));

    shared_ptr<hittable> box2 = make_shared<box>(point3(0,0,0), point3(165,165,165), white);
    box2 = make_shared<rotate_y>(box2, -18);
    box2 = make_shared<translate>(box2, vec3(130,0,65));

    objects.add(make_shared<constant_medium>(box1, 0.01, color(0,0,0)));
    objects.add(make_shared<constant_medium>(box2, 0.01, color(1,1,1)));

    return objects;
}


hittable_list final_scene() {
    hittable_list boxes1;
    auto ground = make_shared<lambertian>(color(0.48, 0.83, 0.53));

    const int boxes_per_side = 20;
    for (int i = 0; i < boxes_per_side; i++) {
        for (int j = 0; j < boxes_per_side; j++) {
            auto w = 100.0;
            auto x0 = -1000.0 + i*w;
            auto z0 = -1000.0 + j*w;
            auto y0 = 0.0;
            auto x1 = x0 + w;
            auto y1 = random_double(1,101);
            auto z1 = z0 + w;

            boxes1.add(make_shared<box>(point3(x0,y0,z0), point3(x1,y1,z1), ground));
        }
    }

    hittable_list objects;

    objects.add(make_shared<bvh_node>(boxes1, 0, 1));

    auto light = make_shared<diffuse_light>(color(7, 7, 7));
    objects.add(make_shared<xz_rect>(123, 423, 147, 412, 554, light));

    auto center1 = point3(400, 400, 200);
    auto center2 = center1 + vec3(30,0,0);
    auto moving_sphere_material = make_shared<lambertian>(color(0.7, 0.3, 0.1));
    objects.add(make_shared<moving_sphere>(center1, center2, 0, 1, 50, moving_sphere_material));

    objects.add(make_shared<sphere>(point3(260, 150, 45), 50, make_shared<dielectric>(1.5)));
    objects.add(make_shared<sphere>(
        point3(0, 150, 145), 50, make_shared<metal>(color(0.8, 0.8, 0.9), 1.0)
    ));

    auto boundary = make_shared<sphere>(point3(360,150,145), 70, make_shared<dielectric>(1.5));
    objects.add(boundary);
    objects.add(make_shared<constant_medium>(boundary, 0.2, color(0.2, 0.4, 0.9)));
    boundary = make_shared<sphere>(point3(0,0,0), 5000, make_shared<dielectric>(1.5));
    objects.add(make_shared<constant_medium>(boundary, .0001, color(1,1,1)));

    auto emat = make_shared<lambertian>(make_shared<image_texture>("earthmap.jpg"));
    objects.add(make_shared<sphere>(point3(400,200,400), 100, emat));
    auto pertext = make_shared<noise_texture>(0.1);
    objects.add(make_shared<sphere>(point3(220,280,300), 80, make_shared<lambertian>(pertext)));

    hittable_list boxes2;
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    int ns = 1000;
    for (int j = 0; j < ns; j++) {
        boxes2.add(make_shared<sphere>(point3::random(0,165), 10, white));
    }

    objects.add(make_shared<translate>(
        make_shared<rotate_y>(
            make_shared<bvh_node>(boxes2, 0.0, 1.0), 15),
            vec3(-100,270,395)
        )
    );

    return objects;
}


int main() {

    auto aspect_ratio = 16.0 / 9.0; // 图像比例
    int image_width = 1280; // 图像宽度
    int samples_per_pixel = 1; // 每像素采样数
    int max_depth = 50; // 最大碰撞深度

    // World
    hittable_list world; // 碰撞体的集合——世界

    point3 lookfrom; // 视点原点
    point3 lookat; // 视点方向
    auto vfov = 40.0; // 视角
    auto aperture = 0.0; // 光圈
    color background(0,0,0); // 背景颜色
    auto sky_box = make_sky_box(); // 天空盒实现

    // 选择对应的场景进行渲染
    switch (2) {
        case 1:
            world = random_scene();
            background = color(0.70, 0.80, 1.00);
            lookfrom = point3(13,2,3);
            lookat = point3(0,0,0);
            vfov = 20.0;
            aperture = 0.1;
            break;

        case 2:
            world = my_scene1();
            background = color(0.70, 0.80, 1.00);
//            lookfrom = point3(13,3,3);
//            lookat = point3(0,0,0);
            lookfrom = point3(7,3,0);
            lookat = point3(0,0,0);
            samples_per_pixel = 100;
            max_depth = 25;
            vfov = 75.0;
            break;

        case 3:
            world = two_perlin_spheres();
            background = color(0.70, 0.80, 1.00);
            lookfrom = point3(13,2,3);
            lookat = point3(0,0,0);
            vfov = 20.0;
            break;

        case 4:
            world = earth();
            background = color(0.70, 0.80, 1.00);
            lookfrom = point3(0,0,12);
            lookat = point3(0,0,0);
            vfov = 20.0;
            break;

        case 5:
            world = simple_light();
            samples_per_pixel = 400;
            lookfrom = point3(26,3,6);
            lookat = point3(0,2,0);
            vfov = 20.0;
            break;

        default:
        case 6:
            world = cornell_box();
            aspect_ratio = 1.0;
            image_width = 600;
            samples_per_pixel = 200;
            lookfrom = point3(278, 278, -800);
            lookat = point3(278, 278, 0);
            vfov = 40.0;
            break;

        case 7:
            world = cornell_smoke();
            aspect_ratio = 1.0;
            image_width = 600;
            samples_per_pixel = 200;
            lookfrom = point3(278, 278, -800);
            lookat = point3(278, 278, 0);
            vfov = 40.0;
            break;

        case 8:
            world = final_scene();
            aspect_ratio = 1.0;
            image_width = 800;
//            samples_per_pixel = 10000;
            samples_per_pixel = 100;
            lookfrom = point3(478, 278, -600);
            lookat = point3(278, 278, 0);
            vfov = 40.0;
            break;
    }

    // 相机
    const vec3 vup(0,1,0); // 相机正向
    const auto dist_to_focus = 10.0; // 焦距
    const int image_height = static_cast<int>(image_width / aspect_ratio); // 渲染图像高度

    camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, 0.0, 1.0); // 生成相机对象，以实现光线生成

    // 渲染
    std::vector<color> framebuffer(image_width * image_height); // 渲染的buffer，以供并行渲染
    boost::timer t_ogm;
#pragma omp parallel for collapse(2) schedule(dynamic, 8) num_threads(6)
    for (int j = image_height - 1; j >= 0 ; j--) {
        for (int i = 0; i < image_width; ++i) {
            color pixel_color(0,0,0);
            for (int s = 0; s < samples_per_pixel; ++s) {
                auto u = (i + random_double()) / (image_width-1);
                auto v = (j + random_double()) / (image_height-1);
                ray r = cam.get_ray(u, v);
//                pixel_color += ray_color(r, background, world, max_depth); // Background 渲染
                pixel_color += ray_color_sky_box(r, sky_box, world, max_depth); // 天空盒渲染
            }
            framebuffer[(image_height - j - 1) * image_width + i] = pixel_color;
        }
    }
    float time_cost = t_ogm.elapsed();
    std::cout << "Time_cost: " << time_cost << std::endl;
    // 渲染结束

    // 从Buffer转为图像显示并保存
    int h = image_height;
    int w = image_width;
    cv::Mat image(h, w,CV_8UC3);
    cv::Mat image2(h, w,CV_8UC3);
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
        color[0] = (unsigned char)(static_cast<int>(256 * clamp(r, 0.0, 0.999)));
        color[1] = (unsigned char)(static_cast<int>(256 * clamp(g, 0.0, 0.999)));
        color[2] = (unsigned char)(static_cast<int>(256 * clamp(b, 0.0, 0.999)));
        image.at<cv::Vec3b>(i/image_width, i%image_width)[0] = color[2];
        image.at<cv::Vec3b>(i/image_width, i%image_width)[1] = color[1];
        image.at<cv::Vec3b>(i/image_width, i%image_width)[2] = color[0];
    }
    cv::imwrite("./scene2.jpg", image);
    cv::imshow("test", image);
    cv::waitKey();
    // 结束
    std::cout << "Image width: " << image_width << std::endl; 
    std::cout << "Image height: " << image_height << std::endl;
    std::cout << "Samples per pixel: " << samples_per_pixel << std::endl;
}
