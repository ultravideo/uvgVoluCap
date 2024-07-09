#ifndef GRIP_HPP
#define GRIP_HPP

#include <glm/glm.hpp>

namespace uvgvolucap {
    namespace geometry {

        class Grid {
        private:
            int max_grid_coord = 0; // Maximum grid coordinate
            float adjusted_grid_dim = 0.0f; // Adjusted grid dimension
            float real_world_range_xy = 0.0;  // from -2.0 to 2.0 for x and y
            float real_world_range_z = 0.0;   // from -0.7 to 2.0 for z

            //Nodes
            float node_size; 

            //Real world max size of the grid
            float real_world_max_horiz = 2.0f;     //x/y in real world
            float real_world_min_horiz = -2.0f;	//x/y in real world
            float real_world_max_vertic = 1.0f;  	//z in real world
            float real_world_min_vertic = -1.0f;		//z in real world

        public:
            Grid(int GP);
            ~Grid();

            void set_real_world_params(float max_xy, float min_xy, float max_z, float min_z);

            glm::vec3 real_to_grid(float real_x, float real_y, float real_z);
        };

    } // namespace geometry
} // namespace uvgvolucap

#endif // GRIP_HPP 