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
#define COLOR_UCHAR

namespace uvgvolucap {
    namespace geometry {  
        /**
         * @brief Struct representing a 3D vector with unsigned 8-bit integer components.
         * 
         * */
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

        /** 
        * @brief Struct representing a 3D vector with unsigned 8-bit integer components.
        **/
        typedef struct vec3u8 {
            uint8_t x;
            uint8_t y;
            uint8_t z;
        
            //default constructor
            vec3u8() : x(0), y(0), z(0) {}
        
            //constructor
            vec3u8(uint8_t _x, uint8_t _y, uint8_t _z) : x(_x), y(_y), z(_z) {}
        } vec3u8;

        /**
         * @brief Struct representing a 3D vector with unsigned 16-bit integer components.
         * 
         * */
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
		 * @brief Class representing a point cloud.
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

            /**
             * @brief Clear everything in the point cloud.
            */
            void clear();

            /**
             * @brief Resize the point cloud.
             * @param size The new size of the point cloud.
            */
			void resize(size_t size);
            
            /**
             * @brief Add a point to the point cloud.
             * @param x The x coordinate of the point.
             * @param y The y coordinate of the point.
             * @param z The z coordinate of the point.
             * @param r The red component of the point.
             * @param g The green component of the point.
             * @param b The blue component of the point.
            */
            void add_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);

            /**
             * @brief This function is called when the point cloud is done with adding points, since the mechanism of resizing the vector in add_point is optimized for performance.
            */
			virtual void finallized() = 0;	

#ifdef POINT_UINT16
            /**
             * @brief Get the position of a point in the point cloud by index.
             * @param index The index of the point.
             * @return The data of x and y and z of the point in unsigned 16-bit integer format.
            */
            const vect3u16& get_position_by_index(size_t index) const;

            /**
             * @brief Get the positions of all the points in the point cloud.
             * @return The container of the positions of all the points in the point cloud.
            */
            const _points_vec3u16_ptr getPositionsVec() const;
#else
            /**
             * @brief Get the position of a point in the point cloud by index.
             * @param index The index of the point.
             * @return The data of x and y and z of the point in float format.
            */
            const glm::vec3& get_position_by_index(size_t index) const;

            /**
             * @brief Get the positions of all the points in the point cloud.
             * @return The container of the positions of all the points in the point cloud.
            */
            const _points_vec3_ptr getPositionsVec() const;
#endif

#ifdef COLOR_UCHAR
            /**
             * @brief Get the attribute of a point in the point cloud by index.
             * @param index The index of the point.
             * @return The data of r and g and b of the point in unsigned 8-bit integer format.
            */
            const vec3u8& get_attribute_by_index(size_t index) const;

            /**
             * @brief Get the attributes of all the points in the point cloud.
             * @return The container of the attributes of all the points in the point cloud.
            */
			const _attributes_vec3u8_ptr getAttributesVec() const;
#else
            /**
             * @brief Get the attribute of a point in the point cloud by index.
             * @param index The index of the point.
             * @return The data of r and g and b of the point in float format.
            */
            const glm::vec3& get_attribute_by_index(size_t index) const;

            /**
             * @brief Get the attributes of all the points in the point cloud.
             * @return The container of the attributes of all the points in the point cloud.
            */
			const _attributes_vec3f_ptr getAttributesVec() const;
#endif

            /**
             * @brief Get the maximum size of the point cloud.
             * @return The maximum size of the point cloud.
            */
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

                /** 
                 * @brief Prepares the fragment to be merged into the merge buffer.
                */
                bool prep_to_merge_buffer(std::shared_ptr<MergeBufferPointCloud> _merge_buffer);

                /** 
                 * @brief Copies the fragment to the merge buffer.
                */
                void copy_to_merge_buffer();

                /** 
                 * @brief Sets the maximum bound of the subspace.
                 * @param x The x coordinate of the maximum bound.
                 * @param y The y coordinate of the maximum bound.
                 * @param z The z coordinate of the maximum bound.
                */
                void set_min_bound(int x, int y, int z);

                /** 
                 * @brief Sets the minimum bound of the subspace.
                 * @param x The x coordinate of the minimum bound.
                 * @param y The y coordinate of the minimum bound.
                 * @param z The z coordinate of the minimum bound.
                */
                void set_max_bound(int x, int y, int z);

                /** 
                 * @brief Adds a point to the slice fragment and filter again in the subspace using the distance from the point to the origin.
                 * @param x The x coordinate of the point.
                 * @param y The y coordinate of the point.
                 * @param z The z coordinate of the point.
                 * @param r The red component of the point.
                 * @param g The green component of the point.
                 * @param b The blue component of the point.
                 * @param origin The origin of the subspace.
                */
                void add_point_subspace(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b, glm::vec3 origin);
#ifdef COLOR_UCHAR
                
                void voxlelize(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);
#else
                void voxlelize(float x, float y, float z, float r, float g, float b);
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