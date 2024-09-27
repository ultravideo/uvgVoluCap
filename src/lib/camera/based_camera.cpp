#include "based_camera.hpp"

namespace uvgvolucap {
    namespace camera{
        BasedFrame::BasedFrame(int _id, int min_bound_[3], int max_bound_[3], int number_of_slices_) 
        :  id(_id), 
            min_bound{min_bound_[0], min_bound_[1], min_bound_[2]}, 
            max_bound{max_bound_[0], max_bound_[1], max_bound_[2]}, 
            number_of_slices(number_of_slices_) {

            for (int i = 0; i < number_of_slices; i++)
            {
                std::shared_ptr<geometry::PclFragment> subspace_slice = std::make_shared<geometry::PclFragment>();
                subspace_slice->set_min_bound(min_bound[0], i * (max_bound[1] - min_bound[1]) / number_of_slices, min_bound[2]);
                subspace_slice->set_max_bound(max_bound[0], (i + 1) * (max_bound[1] - min_bound[1]) / number_of_slices, max_bound[2]);
                subspace_fragments->push_back(subspace_slice);
            }
        }

        BasedFrame::~BasedFrame() {
            subspace_fragments->clear();
            subspace_fragments.reset();
            fragment_pcl.reset();
        }

        int BasedFrame::get_number_of_slices() {
            return number_of_slices;
        }

        int BasedFrame::get_max_bound(int index) {
            return max_bound[index];
        }

        int BasedFrame::get_min_bound(int index) {
            return min_bound[index];
        }

        geometry::_slice_fragments_ptr BasedFrame::get_subspace_fragments() {
            return subspace_fragments;
        }

        std::shared_ptr<geometry::PclFragment> BasedFrame::get_frame_pointcloud() {
            return fragment_pcl;
        }

        void BasedFrame::set_min_bound(int x, int y, int z) {
            min_bound[0] = x;
            min_bound[1] = y;
            min_bound[2] = z;
        }

        void BasedFrame::set_max_bound(int x, int y, int z) {
            max_bound[0] = x;
            max_bound[1] = y;
            max_bound[2] = z;
        }

        void BasedFrame::set_number_of_slices(int num) {
            number_of_slices = num;
        }
    }
}