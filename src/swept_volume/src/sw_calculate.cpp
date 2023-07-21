#include <swept_volume/sw_calculate.hpp>
#include <unordered_map>
 constexpr std::array<uint8_t, 30> sw_calculate::zv;
 constexpr std::array<std::array<int, 4>, 30> sw_calculate::zvv;
void sw_calculate::calculation(const Eigen::RowVector3d &p0, const std::vector<Eigen::RowVector3i> &init_voxels,
                               const std::vector<double> &t0, const std::function<double(const Eigen::RowVector3d &, double &, std::vector<std::vector<double>> &, std::vector<std::vector<double>> &, std::vector<std::vector<double>> &)> scalarFunc, const double eps,
                               const int expected_number_of_cubes)
{
    struct IndexRowVectorHash
    {
        std::size_t operator()(const Eigen::RowVector3i &key) const
        {
            std::size_t seed = 0;
            std::hash<int> hasher;
            for (int i = 0; i < 3; i++)
            {
                seed ^= hasher(key[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
    double half_eps = 0.5 * eps;
    CI_vector.reserve(expected_number_of_cubes);
    CV_vector.reserve(8 * expected_number_of_cubes);
    CS_vector.reserve(8 * expected_number_of_cubes);
    argmins.reserve(32 * expected_number_of_cubes);
    int counter = 0;

    std::unordered_map<Eigen::RowVector3i,int, IndexRowVectorHash> visited;
    visited.reserve(6 * expected_number_of_cubes);
    visited.max_load_factor(0.5);
    queue.reserve(expected_number_of_cubes * 8);
    time_queue.reserve(expected_number_of_cubes * 8);
    correspondence_queue.reserve(expected_number_of_cubes * 8);

    for (int seed_ind = 0; seed_ind < init_voxels.size(); seed_ind++)
    {

        double min_turn = 1000.0;
        double final_seed = t0[seed_ind];

        queue.push_back(init_voxels[seed_ind]);
        time_queue.push_back(final_seed);
        correspondence_queue.push_back(-1);
    }
    const int totalseeds=queue.size();
    int marchsize=0;
    int additions_normal, additions_corrections, additions_self;
    additions_normal = 0;
    additions_corrections = 0;
    additions_self = 0;
    while (queue.size() > 0)
    {

        Eigen::RowVector3i pi = queue.back();
        queue.pop_back();
        double time_seed = time_queue.back();
        ///////////for percentage show/////////////
        marchsize++;
        if(marchsize>=totalseeds/2)
        {
        marchsize=0;
        std::cout<<"SW calculate percentage:"<<(time_seed/t_max)*100<<"%"<<std::endl;
        }
        ///////////for percentage show/////////////
        
        time_queue.pop_back();

        int correspondence = correspondence_queue.back();
        correspondence_queue.pop_back();
        Eigen::RowVector3d ctr = p0 + eps * pi.cast<double>();
        std::array<Eigen::RowVector3d, 8> cubeCorners = {
            ctr + half_eps * (bx + by + bz).cast<double>(), ctr + half_eps * (bx + by - bz).cast<double>(), ctr + half_eps * (-bx + by - bz).cast<double>(), ctr + half_eps * (-bx + by + bz).cast<double>(),
            ctr + half_eps * (bx - by + bz).cast<double>(), ctr + half_eps * (bx - by - bz).cast<double>(), ctr + half_eps * (-bx - by - bz).cast<double>(), ctr + half_eps * (-bx - by + bz).cast<double>()};
        double time_test;
        argmins[CI_vector.size()].resize(8);
        std::vector<double> argmins_cube;
        uint8_t vertexAlreadyAdded = 0;
        bool flag = false;

        Eigen::Matrix<int, 1, 8> cube;
        cube << -1, -1, -1, -1, -1, -1, -1, -1;
        for (int n = 0; n < 30; n++)
        {
            Eigen::RowVector3i nkey = pi + neighbors[n];
            auto nbr = visited.find(nkey);
            if (nbr != visited.end())
            {
                for (int i = 0; i < 4; i++)
                {
                    if (zvv[n][i] != -1)
                    {

                        cube[zvv[n][i]] = CI_vector[nbr->second][zvv[n % 2 == 0 ? n + 1 : n - 1][i]];
                    }
                }
            }
        }

        bool we_in = true;
        for (int i = 0; i < 8; i++)
        {
            if (cube[i] == -1)
            {
                we_in = false;
                break;
            }
            if (CS_vector[cube[i]] > 0.0)
            {
                we_in = false;
                break;
            }
        }

        if (we_in)
        {
            continue;
        }

        std::vector<std::vector<std::vector<double>>> intervals;
        intervals.resize(8);
        std::vector<std::vector<std::vector<double>>> values;
        values.resize(8);
        std::vector<std::vector<std::vector<double>>> minima;
        minima.resize(8);

        bool debug_flag = false;
        bool in_existing_interval = false;
        bool intersecting_interval = false;
        time_test = time_seed; 
        double running_argmin = 0.0;

        for (int i = 0; i < 8; i++)
        {
            time_test = time_seed;

            if (cube[i] >= 0)
            {
                cubeScalars[i] = scalarFunc(cubeCorners[i], time_test, CV_intervals[cube[i]], CV_values[cube[i]], CV_minima[cube[i]]);
                argmins[CI_vector.size()][i] = time_test;

                if (correspondence == -1)
                {
                    double temp = cubeScalars[i];

                    int temp_i = -1;
                    int temp_s = -1;
                    for (int s = 0; s < CV_intervals[cube[i]].size(); s++)
                    {
                        for (int mm = 0; mm < (CV_intervals[cube[i]][s].size() / 2); mm++)
                        {
                            if ((CV_values[cube[i]][s][mm] + 1e-3) < temp)
                            {
                                temp = CV_values[cube[i]][s][mm];
                                temp_i = mm;
                                temp_s = s;
                            }
                        }
                    }
                    if (temp_i > -1)
                    {
                        queue.push_back(pi);
                        time_queue.push_back(CV_minima[cube[i]][temp_s][temp_i]);
                        correspondence_queue.push_back(1);

                        additions_self++;
                    }
                }
            }
            else
            {
                time_test = time_seed;
                intervals[i].resize(0);
                values[i].resize(0);
                minima[i].resize(0);
                cubeScalars[i] = scalarFunc(cubeCorners[i], time_test, intervals[i], values[i], minima[i]);
                argmins[CI_vector.size()][i] = time_test;
            }

            running_argmin = running_argmin + (argmins[CI_vector.size()][i] / 8.0);
        }

        bool validCube = false;
        int sign = sgn(cubeScalars[0]);
        for (int i = 1; i < 8; i++)
        {
            if (sign != sgn(cubeScalars[i]))
            {
                validCube = true;
                break;
            }
        }

        for (int n = 0; n < 30; n++)
        {
            Eigen::RowVector3i nkey = pi + neighbors[n];
            auto nbr = visited.find(nkey);
            flag = false;
            if (nbr != visited.end())
            {
                vertexAlreadyAdded |= zv[n];
                for (int i = 0; i < 4; i++)
                {
                    if (zvv[n][i] != -1)
                    {
                        cube[zvv[n][i]] = CI_vector[nbr->second][zvv[n % 2 == 0 ? n + 1 : n - 1][i]];

                        if ((CS_vector[cube[zvv[n][i]]] > cubeScalars[zvv[n][i]] && (CS_vector[cube[zvv[n][i]]] * cubeScalars[zvv[n][i]]) < 0) || CS_vector[cube[zvv[n][i]]] > (cubeScalars[zvv[n][i]] + 1e-3))
                        {
                            if (!flag)
                            {
                                queue.push_back(nkey);
                                time_queue.push_back(running_argmin);
                                correspondence_queue.push_back(nbr->second);
                                additions_corrections++;
                                flag = true;
                            }
                        }
                    }
                }
            }
        }

        validCube = false;
        sign = sgn(cubeScalars[0]);
        for (int i = 1; i < 8; i++)
        {
            if (sign != sgn(cubeScalars[i]))
            {
                validCube = true;
            }
        }
        bool validCube_before = validCube;

        for (int n = 0; n < 30; n++)
        {
            Eigen::RowVector3i nkey = pi + neighbors[n];
            auto nbr = visited.find(nkey);
            flag = false;
            if (nbr != visited.end())
            {
                for (int i = 0; i < 4; i++)
                {
                    if (zvv[n][i] != -1)
                    {
                        cube[zvv[n][i]] = CI_vector[nbr->second][zvv[n % 2 == 0 ? n + 1 : n - 1][i]];

                        cubeScalars[zvv[n][i]] = std::min(CS_vector[cube[zvv[n][i]]], cubeScalars[zvv[n][i]]);
                        CS_vector[cube[zvv[n][i]]] = cubeScalars[zvv[n][i]];
                    }
                }
            }
        }

        validCube = false;
        sign = sgn(cubeScalars[0]);
        for (int i = 1; i < 8; i++)
        {
            if (sign != sgn(cubeScalars[i]))
            {
                validCube = true;
            }
        }

        for (int n = 0; n < 6; n++)
        {
            Eigen::RowVector3i nkey = pi + neighbors[n];
            auto nbr = visited.find(nkey);
            flag = false;
            if (nbr == visited.end())
            {
                if (validCube && validCube_before)
                {
                    queue.push_back(nkey);
                    time_queue.push_back(running_argmin);
                    correspondence_queue.push_back(-1);

                    additions_normal++;
                }
            }
        }

        auto did_we_visit_this_one = visited.find(pi);
        if (correspondence == -1 && did_we_visit_this_one == visited.end())
        {
            for (int i = 0; i < 8; i++)
            {
                if (0 == ((1 << i) & vertexAlreadyAdded))
                {
                    CV_intervals.push_back(intervals[i]);
                    CV_values.push_back(values[i]);
                    CV_minima.push_back(minima[i]);
                    cube[i] = CS_vector.size();
                    CV_vector.push_back(cubeCorners[i]);
                    CS_vector.push_back(cubeScalars[i]);
                }
            }

            visited[pi] = CI_vector.size();
            CI_vector.push_back(cube);
        }
    }

    return;
}
