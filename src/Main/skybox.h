//
// Created by Qiuzhe on 2021/6/16.
//

#ifndef RTWEEKEND_SKYBOX_H
#define RTWEEKEND_SKYBOX_H
#include "rtweekend.h"

#include "aarect.h"
#include "hittable_list.h"


class sky_box : public hittable  {
public:
    sky_box() {}
    sky_box(std::array<shared_ptr<material>, 6> sky_box_ptr);

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        output_box = aabb(box_min, box_max);
        return true;
    }

public:
    point3 box_min;
    point3 box_max;
    hittable_list sides;
};


sky_box::sky_box(std::array<shared_ptr<material>, 6> sky_box_ptr) {

    sides.add(make_shared<xy_rect>(-1, 1, -1, 1, 1, sky_box_ptr[0]));
    sides.add(make_shared<xy_rect>(-1, 1, -1, 1, -1, sky_box_ptr[1]));

    sides.add(make_shared<xz_rect>(-1, 1, -1, 1, 1, sky_box_ptr[2]));
    sides.add(make_shared<xz_rect>(-1, 1, -1, 1, -1, sky_box_ptr[3]));

    sides.add(make_shared<yz_rect>(-1, 1, -1, 1, 1, sky_box_ptr[4]));
    sides.add(make_shared<yz_rect>(-1, 1, -1, 1, -1, sky_box_ptr[5]));
}

bool sky_box::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    return sides.hit(r, t_min, t_max, rec);
}

#endif //RTWEEKEND_SKYBOX_H
