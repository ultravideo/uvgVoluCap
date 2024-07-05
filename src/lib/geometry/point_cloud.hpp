#pragma once
#ifndef UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP
#define UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP

#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <mutex>

namespace uvgvolucap {
    namespace geometry {
        /**
		 * @brief Union representing a vector in BGRA format or as an array.
		 */
		typedef union
		{
			struct _bgra
			{
				uint8_t b; /**< b component of a vector */
				uint8_t g; /**< g component of a vector */
				uint8_t r; /**< r component of a vector */
				uint8_t a; /**< a component of a vector */
			} bgra; /**< B, G, R, A representation of a vector */
			uint8_t v[4]; /**< Array representation of a vector */
		} _bgra_t;

        typedef std::vector<glm::vec3> _points_vec3;
        typedef std::vector<glm::vec3> _attributes_vec3;    
        typedef std::shared_ptr<_points_vec3> _points_vec3_ptr;
        typedef std::shared_ptr<_attributes_vec3> _attributes_vec3_ptr;

        /**
		 * @brief Class representing a OpenGL point cloud.
		 */
		class PointCloud
		{
		protected:
			bool has_points() const;
			bool is_empty() const;

			size_t m_size = 0;
			size_t m_max_size = 200000;

            _points_vec3_ptr positions  = std::make_shared<_points_vec3>();
			_attributes_vec3_ptr attributes = std::make_shared<_attributes_vec3>();

		public:
			PointCloud() = default;
			~PointCloud() = default;

			/**
			 * @brief Adds another point cloud to the current point cloud.
			 * @param cloud The point cloud to be added.
			 * @return Reference to the updated OpenGL_PointCloud object.
			 */
			PointCloud &operator+=(const PointCloud &cloud);

			/**
			 * @brief Adds another point cloud to the current point cloud.
			 * @param cloud The point cloud to be added.
			 * @return The resulting point cloud.
			 */
			PointCloud operator+(const PointCloud &cloud) const;

			void finallized();			
            void clear();
			void resize(size_t size);
            void add_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);
			bool reach_limitsize();
			const glm::vec3& get_position_by_index(size_t index) const;
			const glm::vec3& get_attribute_by_index(size_t index) const;
			const _points_vec3_ptr getPositionsVec() const;
			const _attributes_vec3_ptr getAttributesVec() const;
            size_t max_size() const;
		};

        struct MergeBufferPointCloud {
            int id = 0;
            static const size_t max_size = 900000;
            glm::vec3 positions[max_size];
            glm::vec3 attributes[max_size];

            std::mutex buff_mx;
            size_t curr_index = 0;
        };

        class PclFragment : public PointCloud
        {
            private:
                std::shared_ptr<MergeBufferPointCloud> m_merge_buffer = nullptr;
                size_t curr_start_buff_index = 0;
				// _points::iterator curr_pos_iterator = m_merge_buffer->positions->begin();
                // _attributes::iterator curr_attr_iterator = m_merge_buffer->attributes->begin();
            
            public:
                PclFragment();
                ~PclFragment();
                bool prep_to_merge_buffer(std::shared_ptr<MergeBufferPointCloud> _merge_buffer);
                void copy_to_merge_buffer();
        };

    } // namespace geometry
} // namespace uvgvolucap

#endif // UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP