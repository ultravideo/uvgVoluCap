#include "grid.hpp"
#include <cmath>
#include <iostream> 
namespace uvgvolucap {
    namespace geometry { 
        
            Grid::Grid(int GP) {
                max_honz_grid_coord = static_cast<int>(std::pow(2, GP));
                max_vert_grid_coord = static_cast<int>(std::pow(2, GP));
            }

            Grid::~Grid() {

            }

            void Grid::set_real_world_params(float max_xy, float min_xy, float max_z, float min_z) {
                real_world_max_horiz = max_xy;
                real_world_min_horiz = min_xy;
                real_world_max_vertic = max_z;
                real_world_min_vertic = min_z;

                real_world_range_honz = real_world_max_horiz - real_world_min_horiz;
                real_world_range_vert = real_world_max_vertic - real_world_min_vertic;

                // Check if honz range is bigger than vert range and adjust the grid dim
                if (real_world_range_honz > real_world_range_vert) {
                    adjusted_grid_dim = real_world_range_honz - real_world_max_vertic;
                    max_honz_grid_coord = static_cast<int>(((adjusted_grid_dim - real_world_min_horiz) / real_world_range_honz)* max_honz_grid_coord) + max_honz_grid_coord;
                } else {
                    adjusted_grid_dim = real_world_range_vert - real_world_max_horiz;
                    max_vert_grid_coord = static_cast<int>(((adjusted_grid_dim - real_world_min_vertic) / real_world_range_vert)* max_vert_grid_coord) + max_vert_grid_coord;
                }

                origin.x = (((0 - real_world_min_vertic) / real_world_range_vert)* max_vert_grid_coord);
                origin.y = (((0 - real_world_min_horiz) / real_world_range_honz )* max_honz_grid_coord);
                origin.z = (((0 - real_world_min_vertic) / real_world_range_vert)* max_vert_grid_coord);
            }

            glm::vec3 Grid::real_to_grid(float real_x, float real_y, float real_z) {
   
                int grid_x = static_cast<int>(((real_x - real_world_min_vertic) / real_world_range_vert)* max_vert_grid_coord);
                int grid_y = static_cast<int>(((real_y - real_world_min_horiz) / real_world_range_honz )* max_honz_grid_coord);
                int grid_z = static_cast<int>(((real_z - real_world_min_vertic) / real_world_range_vert)* max_vert_grid_coord);

                return glm::vec3(grid_x, grid_y, grid_z);
            }
    }
}