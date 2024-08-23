#pragma once
#ifndef UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP
#define UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP

#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
namespace uvgvolucap {
    namespace geometry {

        /* Test */
        struct VoxelData {
            size_t index;
            int count;
        };

        using VoxelCoord = glm::vec3;

        // Define a hash function for VoxelCoord
        struct VoxelCoordHash {
            std::size_t operator()(const VoxelCoord& coord) const {
                std::size_t hx = std::hash<std::size_t>()(static_cast<std::size_t>(coord.x));
                std::size_t hy = std::hash<std::size_t>()(static_cast<std::size_t>(coord.y));
                std::size_t hz = std::hash<std::size_t>()(static_cast<std::size_t>(coord.z));
                return hx ^ (hy << 1) ^ (hz << 2);  // Combine the hashes
            }
        };

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

            std::mutex add_point_mx;

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

			virtual void finallized() = 0;		
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

        struct MergeBufferPointCloud;

        class PclFragment : public PointCloud
        {
            private:
                std::shared_ptr<MergeBufferPointCloud> m_merge_buffer = nullptr;
                size_t curr_start_buff_index = 0;

                int subspace_min_bound[3] = {0, 0, 0};
                int subspace_max_bound[3] = {0, 0, 0};

                std::unordered_map<VoxelCoord, VoxelData, VoxelCoordHash> voxelMap;
                size_t voxle_map_index = 0;
                std::mutex voxel_mx;
            
            public:
                PclFragment();
                ~PclFragment();
                bool prep_to_merge_buffer(std::shared_ptr<MergeBufferPointCloud> _merge_buffer);
                void copy_to_merge_buffer();
                void set_min_bound(int x, int y, int z);
                void set_max_bound(int x, int y, int z);
                void add_point_subspace(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);

                void voxlelization_add_point(float x, float y, float z, float r, float g, float b);
                void finallized() override;
        };

        struct MergeBufferPointCloud {
            int id = 0;
            static const size_t max_size = 1000000;
            glm::vec3 positions[max_size];
            glm::vec3 attributes[max_size];

            std::mutex buff_mx;
            size_t curr_index = 0;

            std::shared_ptr<std::vector<std::shared_ptr<PclFragment>>> slice_fragments = std::make_shared<std::vector<std::shared_ptr<PclFragment>>>();
            std::shared_ptr<std::vector<std::shared_ptr<std::vector<std::shared_ptr<PclFragment>>>>> slice_components =std::make_shared<std::vector<std::shared_ptr<std::vector<std::shared_ptr<PclFragment>>>>>();

            int step = 8;

            //constructor
            MergeBufferPointCloud(size_t total_cams) {
                for (int i = 0; i < step; i++)
                {
                    std::shared_ptr<PclFragment> subspace_slice = std::make_shared<PclFragment>();
                    slice_fragments->push_back(subspace_slice);
                }

                for (int i = 0; i < total_cams; i++)
                {
                    std::shared_ptr<std::vector<std::shared_ptr<PclFragment>>> slice_container = std::make_shared<std::vector<std::shared_ptr<PclFragment>>>();
                    for (int j = 0; j < step; j++)
                    {
                        std::shared_ptr<PclFragment> subspace_slice = std::make_shared<PclFragment>();
                        slice_container->push_back(subspace_slice);
                    }
                    slice_components->push_back(slice_container);
                }
            }
            
        };

    } // namespace geometry
} // namespace uvgvolucap

#endif // UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP