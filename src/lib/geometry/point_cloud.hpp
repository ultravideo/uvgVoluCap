#pragma once
#ifndef UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP
#define UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP

#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

// #define POINT_UINT16
// #define COLOR_UCHAR

namespace uvgvolucap {
    namespace geometry {
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

        typedef struct vec3u8 {
            uint8_t x;
            uint8_t y;
            uint8_t z;
        
            //default constructor
            vec3u8() : x(0), y(0), z(0) {}
        
            //constructor
            vec3u8(uint8_t _x, uint8_t _y, uint8_t _z) : x(_x), y(_y), z(_z) {}
        } vec3u8;

        typedef struct vect3u16 {
            uint16_t x;
            uint16_t y;
            uint16_t z;
        
            //default constructor
            vect3u16() : x(0), y(0), z(0) {}
        
            //constructor
            vect3u16(uint16_t _x, uint16_t _y, uint16_t _z) : x(_x), y(_y), z(_z) {}
        } vect3u16;

#ifdef POINT_UINT16
        typedef std::vector<vect3u16> _points_vec3u16;
        typedef std::shared_ptr<_points_vec3u16> _points_vec3u16_ptr;
#else
        typedef std::vector<glm::vec3> _points_vec3; 
        typedef std::shared_ptr<_points_vec3> _points_vec3_ptr;
#endif

#ifdef COLOR_UCHAR
        typedef std::vector<vec3u8> _attributes_vec3u8; 
        typedef std::shared_ptr<_attributes_vec3u8> _attributes_vec3u8_ptr;
#else
        typedef std::vector<glm::vec3> _attributes_vec3f;  
        typedef std::shared_ptr<_attributes_vec3f> _attributes_vec3f_ptr;
#endif   

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

#ifdef POINT_UINT16
            _points_vec3u16_ptr positions = std::make_shared<_points_vec3u16>();
#else
            _points_vec3_ptr positions  = std::make_shared<_points_vec3>();
#endif

#ifdef COLOR_UCHAR
            _attributes_vec3u8_ptr attributes = std::make_shared<_attributes_vec3u8>();
#else
			_attributes_vec3f_ptr attributes = std::make_shared<_attributes_vec3f>();
#endif

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

#ifdef POINT_UINT16
            const vect3u16& get_position_by_index(size_t index) const;
            const _points_vec3u16_ptr getPositionsVec() const;
#else
			const glm::vec3& get_position_by_index(size_t index) const;
			const _points_vec3_ptr getPositionsVec() const;
#endif

#ifdef COLOR_UCHAR
            const vec3u8& get_attribute_by_index(size_t index) const;
			const _attributes_vec3u8_ptr getAttributesVec() const;
#else
			const glm::vec3& get_attribute_by_index(size_t index) const;
			const _attributes_vec3f_ptr getAttributesVec() const;
#endif
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
#ifdef COLOR_UCHAR
                void voxlelization_add_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);
#else
                void voxlelization_add_point(float x, float y, float z, float r, float g, float b);
#endif
                void finallized() override;
        };

        typedef std::shared_ptr<std::vector<std::shared_ptr<PclFragment>>> _slice_fragments_ptr;
        typedef std::vector<std::shared_ptr<PclFragment>> _slices_fragment_vec;

        struct MergeBufferPointCloud {
            int id = 0;
            static const size_t max_size = 1000000;

#ifdef POINT_UINT16
            vect3u16 positions[max_size];
#else
            glm::vec3 positions[max_size];
#endif

#ifdef COLOR_UCHAR
            struct vec3u8 attributes[max_size];
#else
            glm::vec3 attributes[max_size];
#endif

            std::mutex buff_mx;
            size_t curr_index = 0;

            std::shared_ptr<std::vector<_slice_fragments_ptr>> slice_components = std::make_shared<std::vector<_slice_fragments_ptr>>();
            int step = 8;

            //constructor
            MergeBufferPointCloud(size_t total_cams) {
                for (int i = 0; i < total_cams; i++)
                {
                    _slice_fragments_ptr device_slice_container = std::make_shared<_slices_fragment_vec>();
                    for (int j = 0; j < step; j++)
                    {
                        std::shared_ptr<PclFragment> subspace_slice = std::make_shared<PclFragment>();
                        device_slice_container->push_back(subspace_slice);
                    }
                    slice_components->push_back(device_slice_container);
                }
            }
            
        };

    } // namespace geometry
} // namespace uvgvolucap

#endif // UVGVOLUCAP_GEOMETRY_POINT_CLOUD_HPP