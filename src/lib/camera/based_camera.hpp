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
            /**
             * @brief Setup the device configuration.
             * @details Different cameras have different configurations. So, providing a virtual method to 
             * setup the device configuration based on the camera API.
             */  
            virtual void init(Args... args) const = 0;    

            /**
             * @brief Open the device.
             * @details Different cameras have different open methods. So, providing a virtual method to open
             * the device based on the camera API.
             */  
            virtual void open() = 0;

            /**
             * @brief Close the device.
             * @details Different cameras have different close methods. So, providing a virtual method to close
             * the device based on the camera API.
             */
            virtual void close() = 0;
            
            /**
             * @brief Start the device.
             * @details Different cameras have different start methods. So, providing a virtual method to start
             * the device based on the camera API.
             */
            virtual void start() = 0;

        public:
            BasedCamera() = default;
            ~BasedCamera() = default;

            /**
             * @brief Warm up the device.
             * @details Depending on the camera, the warm up process can be different.
             */
            virtual void warm_up() = 0;

            /**
             * @brief Stop the device.
             * @details Different cameras have different stop methods. So, providing a virtual method to stop
             * the device based on the camera API.
             */
            virtual void stop() = 0;
        };

    } // namespace camera
} // namespace uvgvolucap


#endif // BASED_CAMERA_HPP