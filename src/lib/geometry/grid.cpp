#include "grid.hpp"
#include <cmath>
#include <iostream> 
namespace uvgvolucap {
    namespace geometry { 
        
            Grid::Grid(int GP) {
                max_grid_coord = static_cast<int>(std::pow(2, GP));
            }

            Grid::~Grid() {

            }

            void Grid::set_real_world_params(float max_xy, float min_xy, float max_z, float min_z) {
                real_world_max_horiz = max_xy;
                real_world_min_horiz = min_xy;
                real_world_max_vertic = max_z;
                real_world_min_vertic = min_z;

                real_world_range_xy = real_world_max_horiz - real_world_min_horiz;
                real_world_range_z = real_world_max_vertic - real_world_min_vertic;

                node_size = (std::max(std::abs(real_world_max_horiz - real_world_min_horiz), std::abs(real_world_max_vertic - real_world_min_vertic)))/max_grid_coord;
            }

            glm::vec3 Grid::real_to_grid(float real_x, float real_y, float real_z) {
                int grid_x = static_cast<int>((real_x - real_world_min_horiz) / node_size);
                int grid_y = static_cast<int>((real_y - real_world_min_horiz) / node_size);
                int grid_z = static_cast<int>((real_z - real_world_min_vertic) / node_size);

                return glm::vec3(grid_x, grid_y, grid_z);
            }
    }
}