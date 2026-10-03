#pragma once

// Loader for the cuboid-array JSON files used by the maze examples (assets/environments/maze/cuboids.json):
// a top-level array of objects with x, y, z, dx, dy, dz (full extents) and optional roll,
// pitch, yaw. Requires nlohmann_json.

#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include <vamp/collision/factory.hh>

namespace experiments
{
    inline auto load_cuboids_json(vamp::collision::Environment<float> &environment, const std::string &path)
        -> bool
    {
        std::ifstream ifs(path);
        if (not ifs.is_open())
        {
            std::cerr << "Failed to open JSON file: " << path << std::endl;
            return false;
        }

        nlohmann::json j;
        try
        {
            ifs >> j;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
            return false;
        }

        if (not j.is_array())
        {
            std::cerr << "Expected top-level JSON array in: " << path << std::endl;
            return false;
        }

        for (const auto &obj : j)
        {
            if (not obj.is_object() or not obj.contains("x") or not obj.contains("y") or
                not obj.contains("z") or not obj.contains("dx") or not obj.contains("dy") or
                not obj.contains("dz"))
            {
                std::cerr << "Skipping element without required fields (x,y,z,dx,dy,dz)" << std::endl;
                continue;
            }

            const std::array<float, 3> position = {
                obj.at("x").get<float>(), obj.at("y").get<float>(), obj.at("z").get<float>()};
            const std::array<float, 3> euler = {
                obj.value("roll", 0.F), obj.value("pitch", 0.F), obj.value("yaw", 0.F)};
            const std::array<float, 3> half_extents = {
                obj.at("dx").get<float>() / 2, obj.at("dy").get<float>() / 2, obj.at("dz").get<float>() / 2};

            environment.cuboids.emplace_back(
                vamp::collision::factory::cuboid::array(position, euler, half_extents));
        }

        return true;
    }
}  // namespace experiments
