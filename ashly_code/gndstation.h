#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <unistd.h>   // write()
#include <errno.h>

std::vector<uint8_t> build_waypoint_packet(uint8_t drone_id,
                                           float lat,
                                           float lon,
                                           float alt);

bool send_packet_fd(int fd, const std::vector<uint8_t>& packet);

bool send_waypoint_fd(int fd, uint8_t drone_id, double lat, double lon, double alt);

void send_locations_fd(int fd,
                       uint8_t drone_id,
                       const std::vector<Location>& locations,
                       float alt);

#endif