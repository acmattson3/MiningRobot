#include <iostream>
#include <fstream>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

int main(int argc, char **argv)
{
    const char *cfg = "sim_marker.txt";
    if (argc > 1) cfg = argv[1];

    MAKE_exchange_marker_reports_depth();

    aurora::vision_marker_reports reps;
    for (auto &r : reps) r.markerID = 0;

    while (true) {
        int id = 1; float x = 1.0f, y = 0.0f, z = 0.0f, yaw = 0.0f;
        std::ifstream f(cfg);
        if (f) {
            f >> id >> x >> y >> z >> yaw;
        }
        auto &r = reps[0];
        r.markerID = id;
        r.coords.origin = vec3(x, y, z);
        r.coords.X = aurora::vec3_from_angle(yaw);
        r.coords.Y = aurora::rotate_90_Z(r.coords.X);
        r.coords.Z = vec3(0, 0, 1);
        r.coords.percent = 100.0f;

        exchange_marker_reports_depth.write_begin() = reps;
        exchange_marker_reports_depth.write_end();

        aurora::data_exchange_sleep(100);
    }
    return 0;
}
