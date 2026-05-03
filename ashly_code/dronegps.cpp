#include "dronegps.h"

using namespace std;

int num_drones = 3;
double relay_ratio = 0.2;
double drone_radius = 400.0;
double drone_diameter = drone_radius * 2; // meters


// *** SEARCH ALGORITHM *** //

double EARTH_RADIUS = 6371000.0; // meters
double PI = 3.14;

// struct Location{
//     double lat;
//     double lon;
// };

// struct Region {
//     Location tl;
//     Location tr;
//     Location br;
//     Location bl;
// };

// struct SearchResult {
//     vector<Location> relay_drones;  // index 0
//     Location search_drone;          // index 1
//     int update_backbone;            // index 2
// };

static Location lerp(const Location& p1, const Location& p2, double t) {
    return {
        p1.lat + (p2.lat - p1.lat) * t,
        p1.lon + (p2.lon - p1.lon) * t
    };
}

double deg2rad(double deg){
    return deg * PI / 180;
}

double rad2deg(double rad){
    return rad * 180.0 / PI;
}

// Calculate distance between search destination (b) and home base (a)
double dist(const Location& a, const Location& b){
    double lat1 = deg2rad(a.lat);
    double lon1 = deg2rad(a.lon);
    double lat2 = deg2rad(b.lat);
    double lon2 = deg2rad(b.lon);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double h = sin(dlat/2)*sin(dlat/2)+cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);

    return 2 * EARTH_RADIUS * atan2(sqrt(h),sqrt(1-h));
}

// Converts points to 3D coordinates on Earth (sphere), moves along arx, converts back to lat/lon
Location interpolate(const Location& a, const Location& b, double fraction) {
    double lat1 = deg2rad(a.lat);
    double lon1 = deg2rad(a.lon);
    double lat2 = deg2rad(b.lat);
    double lon2 = deg2rad(b.lon);

    double d = dist(a, b) / EARTH_RADIUS;

    if (d == 0) return a;

    double A = sin((1 - fraction) * d) / sin(d);
    double B = sin(fraction * d) / sin(d);

    double x = A * cos(lat1) * cos(lon1) + B * cos(lat2) * cos(lon2);
    double y = A * cos(lat1) * sin(lon1) + B * cos(lat2) * sin(lon2);
    double z = A * sin(lat1) + B * sin(lat2);

    double lat = atan2(z, sqrt(x*x + y*y));
    double lon = atan2(y, x);

    return { rad2deg(lat), rad2deg(lon) };
}

vector<Location> create_backbone(Location home, Location dest){
    double distance = dist(home, dest);

    int num_segments = floor(distance / drone_diameter);

    int needed_drones = max(0, num_segments - 1); // number of relay drones, not including search drone

    if(num_drones < needed_drones){
        return {};
    }
    vector<Location> drone_locations;

    for(int i = 1; i <= needed_drones; i++){
        double fraction = static_cast<double>(i) / num_segments;
        drone_locations.push_back(interpolate(home, dest, fraction));
    }

    return drone_locations;
}

vector<Location> update_backbone(Location home, Location dest){
    return {};
}

SearchResult relay_search_drones(const Location& loctl,
                               const Location& loctr,
                               const Location& locbr,
                               const Location& locbl,
                               const Location& home){
    
    double height = (max(loctr.lat, locbr.lat) - min(loctl.lat, locbl.lat)) * 111.32; // km conversion
    double width = (max(loctr.lon, locbr.lon) - min(loctl.lon, locbl.lon)) * 111.32 * cos((loctr.lat - locbr.lat));
    if((width > drone_diameter) || (height > drone_diameter)){ // need to divide into regions to search
        cout << "Need to split into regions" << endl;
        return{ {}, {}, 1};
    }
    // else one search drone can search radius
    // center of region is dest point
    double center_lat = (loctl.lat + loctr.lat + locbr.lat + locbl.lat) / 4;
    double center_lon = (loctl.lon + loctr.lon + locbr.lon + locbl.lon) / 4;
    Location center = {center_lat, center_lon};
    auto result = create_backbone(home, center);
        
    if(result.empty()){
        cout << "Not enough drones" << endl;
        return { {}, center, 1};
    }
    return {result, center, 0};

}

// *** DRONE ASSIGNMENT *** //
// May need to add to GND station code
// enum class DroneRole {
//     Unassigned,
//     Relay,
//     Search
// };

// struct Drone {
//     string mac;
//     DroneRole role;
//     Location target;
// };

void assign_drones(vector<Drone>& drones,
                   const vector<Location>& locations,
                   DroneRole givenrole) {

    int loc_index = 0;

    for (auto& d : drones) {
        if (d.role == DroneRole::Unassigned && loc_index < locations.size()) {
            d.role = givenrole;
            d.target = locations[loc_index];
            loc_index++;
        }
    }
}

// *** SERIAL TO GND *** //
// string role_to_string(DroneRole role) {
//     switch (role) {
//         case DroneRole::Relay:  return "RELAY";
//         case DroneRole::Search: return "SEARCH";
//         default:                return "UNASSIGNED";
//     }
// }

// string build_assign_packet(const Drone& d) {
//     std::ostringstream ss;
//     ss << "ASSIGN," << d.mac << "," << role_to_string(d.role) << ","
//        << std::fixed << std::setprecision(7)
//        << d.target.lat << "," << d.target.lon << "," << 50; // alt
//     return ss.str();
// }

