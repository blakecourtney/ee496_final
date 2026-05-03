#ifndef DRONEGPS_H
#define DRONEGPS_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <optional>
#include <unordered_map>
#include <sstream>
#include <iomanip>

using namespace std;

struct Location {
    double lat;
    double lon;
};

struct Region {
    Location tl;
    Location tr;
    Location br;
    Location bl;
};

enum class DroneRole {
    Unassigned,
    Relay,
    Search
};

struct Drone {
    string mac;
    DroneRole role;
    Location target;
};

struct SearchResult {
    vector<Location> relay_drones;  // index 0
    Location search_drone;          // index 1
    int update_backbone;            // index 2
};

double deg2rad(double deg);
double rad2deg(double rad);
double dist(const Location& a, const Location& b);
Location interpolate(const Location& a, const Location& b, double fraction);

std::vector<Location> create_backbone(Location home, Location dest);
SearchResult relay_search_drones(const Location& loctl,
                                 const Location& loctr,
                                 const Location& locbr,
                                 const Location& locbl,
                                 const Location& home);

void assign_drones(std::vector<Drone>& drones,
                   const std::vector<Location>& locations,
                   DroneRole givenrole);

// std::string role_to_string(DroneRole role);

#endif