#pragma once
#ifndef BASED_CAMERA_HPP
#define BASED_CAMERA_HPP

#include "uvgVoluCap/threadqueue.hpp"

namespace uvgvolucap {
    namespace Camera{
        class BasedCamera {
        protected:
            bool is_opened = false;
            bool is_started = false;
            bool is_voxelized = false;

            std::shared_ptr<uvgvolucap::ThreadQueue> thread_queue;
            virtual void init() = 0;

        public:
            BasedCamera() = default;
            ~BasedCamera() = default;

            virtual void open() = 0;
            virtual void warmup() = 0;
            virtual void start() = 0;
            virtual void stop() = 0;
        };

    } // namespace Camera
} // namespace uvgvolucap


#endif // BASED_CAMERA_HPP