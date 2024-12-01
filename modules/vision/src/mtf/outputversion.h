#pragma once

#include <vector>

class Output_version
{
public:
    enum type
    {
        V1 = 1,
        V2,

        VLAST
    };

    Output_version()
    {
        for (int i = int(V1); i < VLAST; i++) {
            version_vect.push_back(i);
        }
    }

    static Output_version& instance()
    {
        static Output_version global_ov = Output_version();
        return global_ov;
    }

    std::vector<int>& get_versions() { return version_vect; }

private:
    std::vector<int> version_vect;
};
