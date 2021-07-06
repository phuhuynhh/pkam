#ifndef EWOK_RING_BUFFER_INCLUDE_EWOK_ED_NOR_RING_BUFFER_H_
#define EWOK_RING_BUFFER_INCLUDE_EWOK_ED_NOR_RING_BUFFER_H_

#include <ewok/raycast_ring_buffer.h>

#include <deque>
#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace ewok
{
template <int _POW, typename _Datatype = int16_t, typename _Scalar = float, typename _Flag = uint8_t>
class EuclideanDistanceNormalRingBuffer
{
  public:
    static const int _N = (1 << _POW);  // 2 to the power of POW

    // Other definitions
    typedef Eigen::Matrix<_Scalar, 4, 1> Vector4;
    typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<int, 3, 1> Vector3i;
    typedef std::vector<Vector4, Eigen::aligned_allocator<Vector4>> PointCloud;

    typedef std::shared_ptr<EuclideanDistanceNormalRingBuffer<_POW, _Datatype, _Scalar, _Flag>> Ptr;

    EuclideanDistanceNormalRingBuffer(const _Scalar &resolution, const _Scalar &truncation_distance)
      : resolution_(resolution)
      , truncation_distance_(truncation_distance)
      , occupancy_buffer_(resolution)
      , tmp_buffer1_(resolution)
      , tmp_buffer2_(resolution)
      , distance_buffer_(resolution, truncation_distance)
      , norm_buffer_x_(resolution, _Datatype(0))
      , norm_buffer_y_(resolution, _Datatype(0))
      , norm_buffer_z_(resolution, _Datatype(0))
    {
        distance_buffer_.setEmptyElement(std::numeric_limits<_Scalar>::max());
    }

    inline void getIdx(const Vector3 &point, Vector3i &idx) const { distance_buffer_.getIdx(point, idx); }

    inline void getPoint(const Vector3i &idx, Vector3 &point) const { distance_buffer_.getPoint(idx, point); }

    inline Vector3i getVolumeCenter() { return distance_buffer_.getVolumeCenter(); }

    void updateDistance() { compute_edt3d(); }

    void insertPointCloud(const PointCloud &cloud, const Vector3 &origin)
    {
        occupancy_buffer_.insertPointCloud(cloud, origin);

        // // convert to pcl pointcloud, compute normal, and insert

        // // convert to pcl pointcloud
        // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // for(int i = 0; i < int(cloud.size()); ++i)
        // {
        //     pcl::PointXYZ p;
        //     p.x = cloud.at(i)(0);
        //     p.y = cloud.at(i)(1);
        //     p.z = cloud.at(i)(2);
        //     pcl_cloud->points.push_back(p);
        // }

        // // compute normal
        // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        // ne.setInputCloud(pcl_cloud);
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        // ne.setSearchMethod(tree);
        // ne.setRadiusSearch(6 * resolution_);
        // ne.compute(*normals);

        // // concatenate normals and points
        // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        // pcl::concatenateFields(*pcl_cloud, *normals, *cloud_with_normals);

        // // insert normal
        // insertPointCloudNormal(*cloud_with_normals);
    }

    // Add normal updating
    void insertPointCloudNormal(const pcl::PointCloud<pcl::PointNormal> &pn)
    {
        std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal>> points = pn.points;
        // add normal to belonging voxel
        for(int i = 0; i < int(points.size()); ++i)
        {
            Vector3 p;
            Vector3i idx;
            p(0) = points.at(i).x;
            p(1) = points.at(i).y;
            p(2) = points.at(i).z;
            norm_buffer_x_.getIdx(p, idx);
            if(norm_buffer_x_.insideVolume(idx))
            {
                norm_buffer_x_.at(idx) += points.at(i).normal_x;
                norm_buffer_y_.at(idx) += points.at(i).normal_y;
                norm_buffer_z_.at(idx) += points.at(i).normal_z;
            }
        }
        // normalize all normal
        Vector3i off;
        norm_buffer_x_.getOffset(off);
        for(int x = 0; x < _N; x++)
        {
            for(int y = 0; y < _N; y++)
            {
                for(int z = 0; z < _N; z++)
                {
                    Vector3i coord(x, y, z);
                    coord += off;
                    if(occupancy_buffer_.isOccupied(coord))
                    {
                        float norm = sqrt(pow(norm_buffer_x_.at(coord), 2) + pow(norm_buffer_y_.at(coord), 2) +
                                          pow(norm_buffer_z_.at(coord), 2));
                        if(norm < 1e-3)
                            norm_buffer_x_.at(coord) = norm_buffer_y_.at(coord) = norm_buffer_z_.at(coord) = 0.0;
                        else
                        {
                            norm_buffer_x_.at(coord) /= norm;
                            norm_buffer_y_.at(coord) /= norm;
                            norm_buffer_z_.at(coord) /= norm;
                        }

                        // std::cout << "Normal at " << coord.transpose() << " is:" << norm_buffer_x_.at(coord) << ","
                        //           << norm_buffer_y_.at(coord) << "," << norm_buffer_z_.at(coord) << std::endl;
                    }
                }
            }
        }
    }

    // Add normal offset
    virtual void setOffset(const Vector3i &off)
    {
        occupancy_buffer_.setOffset(off);
        distance_buffer_.setOffset(off);
        norm_buffer_x_.setOffset(off);
        norm_buffer_y_.setOffset(off);
        norm_buffer_z_.setOffset(off);
    }

    // Add normal moveVolume
    virtual void moveVolume(const Vector3i &direction)
    {
        occupancy_buffer_.moveVolume(direction);
        distance_buffer_.moveVolume(direction);
        norm_buffer_x_.moveVolume(direction);
        norm_buffer_y_.moveVolume(direction);
        norm_buffer_z_.moveVolume(direction);
    }

    // get ringbuffer as pointcloud
    void getBufferAsCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Vector3d &center)
    {
        // get center of ring buffer
        Vector3i c_idx = getVolumeCenter();
        Vector3 ct;
        getPoint(c_idx, ct);
        center(0) = ct(0);
        center(1) = ct(1);
        center(2) = ct(2);

        // convert ring buffer to point cloud
        Vector3i off;
        norm_buffer_x_.getOffset(off);
        for(int x = 0; x < _N; x++)
        {
            for(int y = 0; y < _N; y++)
            {
                for(int z = 0; z < _N; z++)
                {
                    // only occupied voxel is return
                    Vector3i coord(x, y, z);
                    coord += off;
                    if(occupancy_buffer_.isOccupied(coord))
                    {
                        Vector3 p;
                        getPoint(coord, p);
                        pcl::PointXYZ pclp;
                        pclp.x = p(0);
                        pclp.y = p(1);
                        pclp.z = p(2);
                        cloud.points.push_back(pclp);
                    }
                }
            }
        }
    }

    // Add normal visualizer
    void getMarkerNormal(visualization_msgs::Marker &m)
    {
        // use line list to represent normal
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "normal";
        m.id = 0;
        m.action = visualization_msgs::Marker::MODIFY;
        m.scale.x = 0.03;

        m.color.r = m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 0.5;

        Vector3i off;
        norm_buffer_x_.getOffset(off);

        // add normal to line list
        for(int x = 0; x < _N; x++)
        {
            for(int y = 0; y < _N; y++)
            {
                for(int z = 0; z < _N; z++)
                {
                    // only visualize occupied points' normal
                    Vector3i coord(x, y, z);
                    coord += off;
                    if(!occupancy_buffer_.isOccupied(coord)) continue;

                    geometry_msgs::Point p;
                    // start point of normal, at voxel's center
                    p.x = coord(0) * resolution_;
                    p.y = coord(1) * resolution_;
                    p.z = coord(2) * resolution_;
                    m.points.push_back(p);

                    // end point of normal
                    p.x += 0.5 * norm_buffer_x_.at(coord);
                    p.y += 0.5 * norm_buffer_y_.at(coord);
                    p.z += 0.5 * norm_buffer_z_.at(coord);
                    m.points.push_back(p);
                }
            }
        }
    }

    void getMarkerFree(visualization_msgs::Marker &m) { occupancy_buffer_.getMarkerFree(m); }

    void getMarkerOccupied(visualization_msgs::Marker &m) { occupancy_buffer_.getMarkerOccupied(m); }

    void getMarkerUpdated(visualization_msgs::Marker &m) { occupancy_buffer_.getMarkerUpdated(m); }

    void getMarkerDistance(visualization_msgs::Marker &m, _Scalar distance)
    {
        distance_buffer_.getMarkerHelper(m, "ring_buffer_distance", 0, Vector4(0, 0, 1, 0.5),
                                         [=](const _Scalar &d) { return d <= distance; });
    }

    template <class Derived>
    _Scalar getDistanceWithGrad(const Eigen::MatrixBase<Derived> &point_const,
                                const Eigen::MatrixBase<Derived> &grad_const)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        Eigen::MatrixBase<Derived> &grad = const_cast<Eigen::MatrixBase<Derived> &>(grad_const);

        Vector3 point = point_const.template cast<_Scalar>();
        Vector3 point_m = point.array() - 0.5 * resolution_;

        Vector3i idx;
        distance_buffer_.getIdx(point_m, idx);

        Vector3 idx_point, diff;
        distance_buffer_.getPoint(idx, idx_point);

        diff = (point - idx_point) / resolution_;

        bool all_valid = true;
        _Scalar values[2][2][2];

        for(int x = 0; x < 2 && all_valid; x++)
        {
            for(int y = 0; y < 2 && all_valid; y++)
            {
                for(int z = 0; z < 2 && all_valid; z++)
                {
                    Vector3i current_idx = idx + Vector3i(x, y, z);

                    if(distance_buffer_.insideVolume(current_idx))
                    {
                        values[x][y][z] = distance_buffer_.at(current_idx);
                    }
                    else
                    {
                        all_valid = false;
                    }
                }
            }
        }

        if(all_valid)
        {
            // Trilinear interpolation
            _Scalar v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
            _Scalar v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
            _Scalar v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
            _Scalar v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];

            _Scalar v0 = (1 - diff[1]) * v00 + diff[1] * v10;
            _Scalar v1 = (1 - diff[1]) * v01 + diff[1] * v11;

            _Scalar v = (1 - diff[2]) * v0 + diff[2] * v1;

            grad[2] = (v1 - v0) / resolution_;
            grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) / resolution_;

            grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
            grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
            grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
            grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

            grad[0] /= resolution_;

            return v;
        }
        else
        {
            return truncation_distance_;
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    void compute_edt3d()
    {
        Vector3i offset;
        distance_buffer_.getOffset(offset);

        Vector3i min_vec, max_vec;
        occupancy_buffer_.getUpdatedMinMax(min_vec, max_vec);

        min_vec -= offset;
        max_vec -= offset;

        min_vec.array() -= truncation_distance_ / resolution_;
        max_vec.array() += truncation_distance_ / resolution_;

        min_vec.array() = min_vec.array().max(Vector3i(0, 0, 0).array());
        max_vec.array() = max_vec.array().min(Vector3i(_N - 1, _N - 1, _N - 1).array());

        // ROS_INFO_STREAM("min_vec: " << min_vec.transpose() << " max_vec: " << max_vec.transpose());

        for(int x = min_vec[0]; x <= max_vec[0]; x++)
        {
            for(int y = min_vec[1]; y <= max_vec[1]; y++)
            {
                fill_edt(
                    [&](int z) {
                        return occupancy_buffer_.isOccupied(offset + Vector3i(x, y, z)) ?
                                   0 :
                                   std::numeric_limits<_Scalar>::max();
                    },
                    [&](int z, _Scalar val) { tmp_buffer1_.at(Vector3i(x, y, z)) = val; }, min_vec[2], max_vec[2]);
            }
        }

        for(int x = min_vec[0]; x <= max_vec[0]; x++)
        {
            for(int z = min_vec[2]; z <= max_vec[2]; z++)
            {
                fill_edt([&](int y) { return tmp_buffer1_.at(Vector3i(x, y, z)); },
                         [&](int y, _Scalar val) { tmp_buffer2_.at(Vector3i(x, y, z)) = val; }, min_vec[1], max_vec[1]);
            }
        }

        for(int y = min_vec[1]; y <= max_vec[1]; y++)
        {
            for(int z = min_vec[2]; z <= max_vec[2]; z++)
            {
                fill_edt([&](int x) { return tmp_buffer2_.at(Vector3i(x, y, z)); },
                         [&](int x, _Scalar val) {
                             distance_buffer_.at(offset + Vector3i(x, y, z)) =
                                 std::min(resolution_ * std::sqrt(val), truncation_distance_);
                         },
                         min_vec[0], max_vec[0]);
            }
        }

        occupancy_buffer_.clearUpdatedMinMax();
    }

    template <typename F_get_val, typename F_set_val>
    void fill_edt(F_get_val f_get_val, F_set_val f_set_val, int start = 0, int end = _N - 1)
    {
        int v[_N];
        _Scalar z[_N + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<_Scalar>::max();
        z[start + 1] = std::numeric_limits<_Scalar>::max();

        for(int q = start + 1; q <= end; q++)
        {
            k++;
            _Scalar s;

            do
            {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
                // ROS_INFO_STREAM("k: " << k << " s: " <<  s  << " z[k] " << z[k] << " v[k] " << v[k]);

            } while(s <= z[k]);

            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<_Scalar>::max();
        }

        k = start;

        for(int q = start; q <= end; q++)
        {
            while(z[k + 1] < q) k++;
            _Scalar val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            //      if(val < std::numeric_limits<_Scalar>::max())
            //  ROS_INFO_STREAM("val: " << val << " q: " << q << " v[k] " << v[k]);
            // if(val > truncation_distance_*truncation_distance_) val = std::numeric_limits<_Scalar>::max();
            f_set_val(q, val);
        }
    }

    _Scalar resolution_;
    _Scalar truncation_distance_;

    RaycastRingBuffer<_POW, _Datatype, _Scalar, _Flag> occupancy_buffer_;

    RingBufferBase<_POW, _Scalar, _Scalar> distance_buffer_;

    RingBufferBase<_POW, _Scalar, _Scalar> tmp_buffer1_, tmp_buffer2_;

    // Normal buffer of x, y, z
    RingBufferBase<_POW, _Scalar, _Scalar> norm_buffer_x_;
    RingBufferBase<_POW, _Scalar, _Scalar> norm_buffer_y_;
    RingBufferBase<_POW, _Scalar, _Scalar> norm_buffer_z_;
};
}

#endif  // EWOK_RING_BUFFER_INCLUDE_EWOK_ED_RING_BUFFER_H_
