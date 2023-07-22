//
// Created by qzj on 2021/12/24.
//

#ifndef CMAKE_TEMPLATE_SYSTEM_H
#define CMAKE_TEMPLATE_SYSTEM_H

#include <ros/ros.h>
#include "kitti_loader.h"
#include "resgistrator/Segregator.h"

namespace pagor {

    class kitti_benchmark {

    public:

        kitti_benchmark(const std::string &config_name, int downsample);

        void Run();

    private:
        //config
        YAML::Node config_node_;

        std::shared_ptr<KittiLoader> kitti_loader_ptr_;

        std::shared_ptr<Segregator> segregator_ptr_;
    };
}


#endif //CMAKE_TEMPLATE_SYSTEM_H
