#include "point_cloud.hpp"
#include <iostream>
#include <cmath>	

namespace uvgvolucap {
    namespace geometry {

        void PointCloud::add_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
		{
			glm::vec3 position{x, y, z};
			glm::vec3 color{r/255.0f, g/255.0f, b/255.0f};

			std::lock_guard<std::mutex> lock(add_point_mx);

			m_size++;
			if (positions->size() < m_size){
				positions->resize(positions->size()*2+1);
				attributes->resize(positions->size()*2+1);
			}

			(*positions)[m_size-1] = (position);
			(*attributes)[m_size-1] = (color);
		}
		
		size_t PointCloud::max_size() const
		{
			return positions->size();
		}

		PointCloud &PointCloud::operator+=(const PointCloud &cloud)
		{
			if (cloud.is_empty())
				return (*this);

			size_t old_vert_num = m_size;
			size_t add_vert_num = cloud.max_size();
			size_t new_vert_num = old_vert_num + add_vert_num;

			if (cloud.has_points())
			{
				m_size += add_vert_num;
				if (positions->size() < new_vert_num){
					positions->resize(std::max(positions->size()*2, new_vert_num));
					attributes->resize(std::max(attributes->size()*2, new_vert_num));
				}

				memcpy(positions->data()+old_vert_num, cloud.getPositionsVec()->data(), add_vert_num * sizeof(glm::vec3));
				memcpy(attributes->data()+old_vert_num, cloud.getAttributesVec()->data(), add_vert_num * sizeof(glm::vec3));
			}

			return *this;
		};
		
		const glm::vec3& PointCloud::get_position_by_index(size_t index) const 
		{
			return (*positions)[index];
		}

		const glm::vec3& PointCloud::get_attribute_by_index(size_t index) const 
		{
			return (*attributes)[index];
		}

		const _points_vec3_ptr PointCloud::getPositionsVec() const 
		{
			return positions;
		}

		const _attributes_vec3_ptr PointCloud::getAttributesVec() const 
		{
			return attributes;
		}

		void PointCloud::clear() 
		{
			m_size = 0;
			positions->clear();
			attributes->clear();
		}

		void PointCloud::finallized() 
		{
			positions->resize(m_size);
			attributes->resize(m_size);
		}

		bool PointCloud::reach_limitsize()
		{
			return m_size >= m_max_size;
		}

		bool PointCloud::has_points() const { return positions->size() > 0; };

		bool PointCloud::is_empty() const { return !has_points(); };

        void PointCloud::resize(size_t size) {
            positions->resize(size);
            attributes->resize(size);
        }

        /* ############################################################################################## */

        PclFragment::PclFragment() : PointCloud()
        {
            m_size = 0; 
        }

        PclFragment::~PclFragment() 
        {
            positions->clear();
            attributes->clear();
        }

        bool PclFragment::prep_to_merge_buffer(std::shared_ptr<MergeBufferPointCloud> _merge_buffer)
        {
            m_merge_buffer = _merge_buffer;

            if (m_size == 0)
                return false;
            
            std::lock_guard<std::mutex> lock(m_merge_buffer->buff_mx);

            curr_start_buff_index = m_merge_buffer->curr_index;

            // Check if the remaining buffer is enough
            if (m_merge_buffer->curr_index + m_size > m_merge_buffer->max_size)
            {
                m_size = m_merge_buffer->max_size - m_merge_buffer->curr_index;
            }

            m_merge_buffer->curr_index+=(m_size);

            return true;
        }

        void PclFragment::copy_to_merge_buffer()
        {                
            memcpy(m_merge_buffer->positions + curr_start_buff_index, positions->data(), m_size * sizeof(glm::vec3));
            memcpy(m_merge_buffer->attributes + curr_start_buff_index, attributes->data(), m_size * sizeof(glm::vec3));
			positions->clear();
			attributes->clear();
        }

		void PclFragment::set_max_bound(int x, int y, int z) {
			subspace_max_bound[0] = x;
			subspace_max_bound[1] = y;
			subspace_max_bound[2] = z;
		}

		void PclFragment::set_min_bound(int x, int y, int z) {
			subspace_min_bound[0] = x;
			subspace_min_bound[1] = y;
			subspace_min_bound[2] = z;
		}

		void  PclFragment::add_point_subspace(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
			
			// if (x < subspace_min_bound[0] || x > subspace_max_bound[0] || y < subspace_min_bound[1] || y > subspace_max_bound[1] || z < subspace_min_bound[2] || z > subspace_max_bound[2])
			// 	return;

			// distance betweem x y z to (-25, -25, 377) < 1.1
			if (std::sqrt(std::pow(x + 25, 2) + std::pow(z - 377, 2)) > 300)
				return;
			
			add_point(x, y, z, r, g, b);
		}
    } // namespace geometry
} // namespace uvgvolucap