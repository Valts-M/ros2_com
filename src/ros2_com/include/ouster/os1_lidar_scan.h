 /**
 * @file
 * @brief Holds lidar data by field in column-major order
 */
#ifndef OS1_LIDAR_SCAN_H
#define OS1_LIDAR_SCAN_H

#include <eigen3/Eigen/Eigen>
#include <iterator>

namespace ouster {

	struct LidarScan {
		enum columns { x = 0, y , z, intensity, time, noise, range, altitude, azimuth, count };
		using Data = Eigen::Matrix<double, (uint8_t)columns::count, Eigen::Dynamic, Eigen::RowMajor>;
		using Point = Eigen::Matrix<double, (uint8_t)columns::count, 1 >;

		const ssize_t W;
		const ssize_t H;
		Data data_;
		size_t point_count = W * H;

		LidarScan(size_t w, size_t h) : W(w), H(h), data_{ (uint8_t)columns::count, w * h } {};

		struct iterator;

		iterator begin() { return iterator(0, &this->data_); }

		static inline Point make_val(const double x, const double y, const double z, const uint16_t intensity,
									 const uint64_t time, const uint16_t, const int32_t, const uint16_t noise,
									 const double range, const double altitudeAngle, const double azimuthAngle) {
			Point p;
			// rotate and translate to lidar coordinate system (see OS-1-16 documentation)
			p << -x, -y, z + 0.03618, (double)intensity, time, (double)noise, range, altitudeAngle, azimuthAngle - M_PI;
			return p;
		}

		// Minimal set of operations to support os1_util.h:batch_to_iter; not really
		// a proper iterator. Remove when Eigen support for STL iterators lands.
		struct iterator {
			using iterator_category = std::output_iterator_tag;
			using value_type = LidarScan::Point;
			using difference_type = void;
			using pointer = void;
			using reference = void;

			inline iterator operator++() {
				idx_++;
				return *this;
			}
			inline Data::ColXpr operator*() { return data_->col(idx_); }

			inline Data::ColXpr operator[](int i) { return data_->col(idx_ + i); }

			friend iterator operator+(iterator lhs, int i) {
				return iterator{ lhs.idx_ + i, lhs.data_ };
			}

			friend bool operator==(const iterator& lhs, const iterator& rhs) {
				return lhs.idx_ == rhs.idx_;
			}

			friend bool operator!=(const iterator& lhs, const iterator& rhs) {
				return !(lhs == rhs);
			}

		private:
			iterator(int idx, Data* data) : idx_{ idx }, data_{ data } {}
			int idx_;
			Data* data_;

			friend class LidarScan;
		};
	};
}
#endif // OS1_LIDAR_SCAN_H