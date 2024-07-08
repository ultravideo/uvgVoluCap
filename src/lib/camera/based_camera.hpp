#pragma once
#ifndef BASED_CAMERA_HPP
#define BASED_CAMERA_HPP

#include "uvgVoluCap/threadqueue.hpp"

namespace uvgvolucap {
    namespace camera{
        template <typename... Args>
        class BasedCamera {
        protected:
            bool is_opened_flag = false;
            bool is_started_flag = false;
            bool is_voxelized_flag = false;
            std::shared_ptr<uvgvolucap::ThreadQueue> thread_queue;

        /* ####################################################### */
        
        protected:
            virtual void init(Args... args) const = 0;            
            virtual void open() = 0;
            virtual void close() = 0;
            virtual void start() = 0;

        public:
            BasedCamera() = default;
            ~BasedCamera() = default;

            virtual void warm_up() = 0;
            virtual void stop() = 0;
        };

    } // namespace camera
} // namespace uvgvolucap


#endif // BASED_CAMERA_HPP