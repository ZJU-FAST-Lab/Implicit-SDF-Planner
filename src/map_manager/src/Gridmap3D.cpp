#include "map_manager/GridMap3D.h"

void GridMap3D::clearGridMap()
{

    for (int i = 0; i < totalsize; i++)
    {
        grid_map[i] = 0;
        grid_map_buffer_neg[i] = 0;
        grid_map_buffer_all[i] = 0;
        grid_map_flags[i] = 0;
    }
}

void GridMap3D::releaseMemory()
{

    delete[] grid_map;
    delete[] grid_map_buffer_neg;
    delete[] grid_map_buffer_all;
    delete[] grid_map_flags;

}

void GridMap3D::createGridMap(const Vector3d &boundary_xyzmin, const Vector3d &boundary_xyzmax)
{
    this->boundary_xyzmin = boundary_xyzmin;
    this->boundary_xyzmax = boundary_xyzmax;
    X_size = ceil((boundary_xyzmax(0) - boundary_xyzmin(0)) / grid_resolution);
    Y_size = ceil((boundary_xyzmax(1) - boundary_xyzmin(1)) / grid_resolution);
    Z_size = ceil((boundary_xyzmax(2) - boundary_xyzmin(2)) / grid_resolution);


    totalsize = X_size * Y_size * Z_size;
    grid_map = new double[totalsize](); 
    grid_map_buffer_neg = new double[totalsize]();
    grid_map_buffer_all = new double[totalsize]();
    grid_map_flags = new bool[totalsize]();
}

bool GridMap3D::isInMap(const Vector3d &pos_w)const
{
    if (pos_w(0) < boundary_xyzmin(0))
    {
        return false;
    }
    if (pos_w(1) < boundary_xyzmin(1))
    {
        return false;
    }
    if (pos_w(2) < boundary_xyzmin(2))
    {
        return false;
    }

    if (pos_w(0) > boundary_xyzmax(0))
    {
        return false;
    }
    if (pos_w(1) > boundary_xyzmax(1))
    {
        return false;
    }
    if (pos_w(2) > boundary_xyzmax(2))
    {
        return false;
    }
    return true;
}

bool GridMap3D::isIndexValid(const Vector3i &index)const
{
    if (index(0) < 0)
    {
        return false;
    }
    if (index(0) >= X_size)
    {
        return false;
    }

    if (index(1) < 0)
    {
        return false;
    }
    if (index(1) >= Y_size)
    {
        return false;
    }

    if (index(2) < 0)
    {
        return false;
    }
    if (index(2) >= Z_size)
    {
        return false;
    }

    return true;
}

bool GridMap3D::isIndexValid( const int ix,const int iy,const int iz)const
{
    if (ix < 0)
    {
        return false;
    }
    if (ix >= X_size)
    {
        return false;
    }

    if (iy < 0)
    {
        return false;
    }
    if (iy >= Y_size)
    {
        return false;
    }

    if (iz < 0)
    {
        return false;
    }
    if (iz >= Z_size)
    {
        return false;
    }

    return true;
}

Vector3i GridMap3D::getGridIndex(const Vector3d &pos_w)
{
    if (!isInMap(pos_w))
    {
        return Vector3i(0, 0, 0);
    }

    double dx = pos_w(0) - boundary_xyzmin(0);
    double dy = pos_w(1) - boundary_xyzmin(1);
    double dz = pos_w(2) - boundary_xyzmin(2);
    int ix = floor(dx / grid_resolution);
    int iy = floor(dy / grid_resolution);
    int iz = floor(dz / grid_resolution);

    if (ix < 0)
    {
        ix = 0;
    }
    if (ix >= X_size)
    {
        ix = X_size - 1;
    }
    if (iy < 0)
    {
        ix = 0;
    }
    if (iy >= Y_size)
    {
        iy = Y_size - 1;
    }
    if (iz < 0)
    {
        ix = 0;
    }
    if (iz >= Z_size)
    {
        iz = Z_size - 1;
    }

    return Vector3i(ix, iy, iz);
}

Vector3d GridMap3D::getGridCubeCenter(int ix, int iy, int iz)
{
    return getGridCubeCenter(Vector3i(ix, iy, iz));
}

Vector3d GridMap3D::getGridCubeCenter(const Vector3i &index)
{
    if (!isIndexValid(index))
    {

        return Vector3d(0, 0, 0);
    }
    double dx = (index(0) + 0.5) * grid_resolution;
    double dy = (index(1) + 0.5) * grid_resolution;
    double dz = (index(2) + 0.5) * grid_resolution;
    Vector3d dpos_w = Vector3d(dx, dy, dz);
    return dpos_w + boundary_xyzmin;
}

bool GridMap3D::isIndexOccupied(const Vector3i &index)
{
    if (!isIndexValid(index))
    {

        return true;
    }
    if (grid_map[toAddr(index(0),index(1),index(2))] == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool GridMap3D::isIndexOccupiedFlate(const Vector3i &index, const int flate_pix)
{
    Eigen::Vector3i new_ind = index;
    for (int dx = -flate_pix; dx <= flate_pix; dx++)
    {
        for (int dy = -flate_pix; dy <= flate_pix; dy++)
        {
            for (int dz = -flate_pix; dz <= flate_pix; dz++)
            {
                new_ind(0) = index(0) + dx;
                new_ind(1) = index(1) + dy;
                new_ind(2) = index(2) + dz;
                if (!isIndexValid(new_ind))
                {
                    continue;
                }
                else if (grid_map[toAddr(new_ind(0),new_ind(1),new_ind(2))]== 1)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool GridMap3D::isIndexOccupied(int ix, int iy, int iz)
{
    bool invalid = false;
    if (ix < 0)
    {
        // invalid = true;
        return true;
    }
    if (ix >= X_size)
    {
        // invalid = true;
          return true;
    }
    if (iy < 0)
    {
        // invalid = true;
        return true;
    }
    if (iy >= Y_size)
    {
        // invalid = true;
        return true;
    }
    if (iz < 0)
    {
        // invalid = true;
        return true;
    }
    if (iz >= Z_size)
    {
        // invalid = true;
        return true;
    }
    if (invalid)
    {
        return true;
    }
    if (grid_map[toAddr(ix,iy,iz)]== 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool GridMap3D::isCoordOccupied(const Vector3d &pos_w)
{
    if (!isInMap(pos_w))
    {
        return true;
    }
    return isIndexOccupied(getGridIndex(pos_w));
}

bool GridMap3D::isGridBoundOfObs(const Vector3i &index)
{
    if (!isIndexOccupied(index))
    {
        cout << "[isGridBoundOfObs] this function got a free index!" << endl;
        return false;
    }

    int idx, idy, idz;
    Vector3i index_scan;
    for (idx = 0; idx < X_size; idx++)
    {
        for (idy = 0; idy < Y_size; idy++)
        {
            for (idz = 0; idz < Z_size; idz++)
            {
                index_scan = index + Vector3i(idx, idy, idz);
                if (index_scan == index)
                {
                    continue;
                }
                if (!isIndexValid(index_scan))
                {
                    continue;
                }
                if (!isIndexOccupied(index_scan))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void GridMap3D::clearGridESDF()
{
    for (int i = 0; i < totalsize; i++)
    {
        grid_esdf_buffer1[i]=0;
        grid_esdf_buffer2[i]=0;
    }
}

int GridMap3D::getVoxelNum(int dim)
{
    if (dim == 0)
    {
        return X_size;
    }
    if (dim == 1)
    {
        return Y_size;
    }
    if (dim == 2)
    {
        return Z_size;
    }
    return 0;
}

void debugP(double num)
{
    cout << num << endl;
}

void GridMap3D::generateESDF3d()
{
    Vector3i min_esdf(0, 0, 0);
    Vector3i max_esdf(X_size - 1, Y_size - 1, Z_size - 1);

    grid_esdf = new double[totalsize]();
    grid_esdf_buffer1 = new double[totalsize]();
    grid_esdf_buffer2 = new double[totalsize]();
    // clearGridESDF();

    /* ========== compute positive DT ========== */

    for (int x = min_esdf(0); x <= max_esdf(0); x++)
    {
        for (int y = min_esdf(1); y <= max_esdf(1); y++)
        {
            fillESDF(
                [&](int z)
                {
                    return isIndexOccupied(x, y, z) == 1 ? 0 : std::numeric_limits<double>::max();
                },
                [&](int z, double val)
                { grid_esdf_buffer1[toAddr(x,y,z)] = val; },
                min_esdf(2),
                max_esdf(2), 2);
        }
    }

    for (int x = min_esdf(0); x <= max_esdf(0); x++)
    {
        for (int z = min_esdf(2); z <= max_esdf(2); z++)
        {
            fillESDF([&](int y)
                     { return grid_esdf_buffer1[toAddr(x,y,z)]; },
                     [&](int y, double val)
                     { grid_esdf_buffer2[toAddr(x,y,z)] = val; },
                     min_esdf(1),
                     max_esdf(1), 1);
        }
    }

    for (int y = min_esdf(1); y <= max_esdf(1); y++)
    {
        for (int z = min_esdf(2); z <= max_esdf(2); z++)
        {
            fillESDF([&](int x)
                     { return grid_esdf_buffer2[toAddr(x,y,z)]; },
                     [&](int x, double val)
                     {
                         grid_esdf[toAddr(x,y,z)] = grid_resolution * std::sqrt(val);
                     },
                     min_esdf(0), max_esdf(0), 0);
        }
    }

    /* ========== compute negative distance ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z)
            {

                if (grid_map[toAddr(x,y,z)] == 0)
                {
                    grid_map_buffer_neg[toAddr(x,y,z)] = 1;
                }
                else if (grid_map[toAddr(x,y,z)] == 1)
                {
                    grid_map_buffer_neg[toAddr(x,y,z)] = 0;
                }
                else
                {
                    ROS_ERROR("what?");
                }
            }

    ros::Time t1, t2;

    for (int x = min_esdf(0); x <= max_esdf(0); x++)
    {
        for (int y = min_esdf(1); y <= max_esdf(1); y++)
        {
            fillESDF(
                [&](int z)
                {
                    return grid_map_buffer_neg[toAddr(x,y,z)] == 1 ? 0 : std::numeric_limits<double>::max();
                },
                [&](int z, double val)
                { grid_esdf_buffer1[toAddr(x,y,z)] = val; },
                min_esdf(2),
                max_esdf(2), 2);
        }
    }

    for (int x = min_esdf(0); x <= max_esdf(0); x++)
    {
        for (int z = min_esdf(2); z <= max_esdf(2); z++)
        {
            fillESDF([&](int y)
                     { return grid_esdf_buffer1[toAddr(x,y,z)]; },
                     [&](int y, double val)
                     { grid_esdf_buffer2[toAddr(x,y,z)] = val; },
                     min_esdf(1),
                     max_esdf(1), 1);
        }
    }

    for (int y = min_esdf(1); y <= max_esdf(1); y++)
    {
        for (int z = min_esdf(2); z <= max_esdf(2); z++)
        {
            fillESDF([&](int x)
                     { return grid_esdf_buffer2[toAddr(x,y,z)]; },
                     [&](int x, double val)
                     {
                         grid_map_buffer_neg[toAddr(x,y,z)] = grid_resolution * std::sqrt(val);
                     },
                     min_esdf(0), max_esdf(0), 0);
        }
    }

    /* ========== combine pos and neg DT ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z)
            {

                grid_map_buffer_all[toAddr(x,y,z)] = grid_map[toAddr(x,y,z)];

                if (grid_map_buffer_neg[toAddr(x,y,z)] > 0.0)
                    grid_map_buffer_all[toAddr(x,y,z)] += (-grid_map_buffer_neg[toAddr(x,y,z)] + grid_resolution);
            }
}

template <typename F_get_val, typename F_set_val>
void GridMap3D::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{

    int v[getVoxelNum(dim)];
    double z[getVoxelNum(dim) + 1];

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++)
    {
        k++;
        double s;

        do
        {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);

        k++;

        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++)
    {
        while (z[k + 1] < q)
            k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
    }
}

double GridMap3D::getGridSDFValue(const Vector3i &index)
{
    if (!isIndexValid(index))
    {
        return 0;
    }
    return grid_esdf[toAddr(index(0),index(1),index(2))];
}

double GridMap3D::getGridSDFValue(int ix, int iy, int iz)
{
    if (ix < 0)
    {
        ix = 0;
    }
    if (ix >= X_size)
    {
        ix = X_size - 1;
    }
    if (iy < 0)
    {
        iy = 0;
    }
    if (iy >= Y_size)
    {
        iy = Y_size - 1;
    }
    if (iz < 0)
    {
        iz = 0;
    }
    if (iz >= Z_size)
    {
        iz = Z_size - 1;
    }
    return grid_esdf[toAddr(ix,iy,iz)];
}
