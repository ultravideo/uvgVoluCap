#ifndef GRIP_HPP
#define GRIP_HPP

#include <glm/glm.hpp>

namespace uvgvolucap {
    namespace geometry {

        class Grid {
        private:
            int max_honz_grid_coord = 0; 
            int max_vert_grid_coord = 0; 
            float adjusted_grid_dim = 0.0f; 
            float real_world_range_vert = 0.0;  
            float real_world_range_honz = 0.0; 
            
            glm::vec3 origin = glm::vec3(0.0f, 0.0f, 0.0f);

            //Real world max size of the grid
            float real_world_max_horiz = 2.0f;     //x/y in real world
            float real_world_min_horiz = -2.0f;	//x/y in real world
            float real_world_max_vertic = 1.0f;  	//z in real world
            float real_world_min_vertic = -1.0f;		//z in real world

        public:
            Grid(int GP);
            ~Grid();

            /**
             * @brief Set the real world parameters.
             * @details This function is used to set the real world parameters from the configuration file.
             */
            void set_real_world_params(float max_xy, float min_xy, float max_z, float min_z);

            /**
             * @brief Convert the coordinates from real world to grid world.
             * @details This function take the coordinates in real world from input device and convert them to grid world.
             */
            glm::vec3 real_to_grid(float real_x, float real_y, float real_z);

            /**
             * @brief Take the coordinates of the origin in grid world. 
             * @details Purpose of origin is used to filter the points that are not in the bounding box. (In this case is a cylinder)
             */
            glm::vec3 get_grid_origin() { return origin; }
        };

    } // namespace geometry
} // namespace uvgvolucap

#endif // GRIP_HPP 