// *** SERIAL TO GND *** //
// Binary packet format must match Python:
// [0] 0xFE
// [1] drone_id
// [2] packet_type
// [3..66] 64-byte payload
// [67] 0xFF

#include "dronegps.h"
#include "gndstation.h"

static constexpr uint8_t PACKET_START_BYTE = 0xFE;
static constexpr uint8_t PACKET_END_BYTE   = 0xFF;
static constexpr uint8_t PKT_TYPE_WAYPOINT = 5;
static constexpr size_t  PAYLOAD_SIZE      = 64;
static constexpr size_t  PACKET_SIZE       = 68;

std::vector<uint8_t> build_waypoint_packet(uint8_t drone_id,
                                           float lat,
                                           float lon,
                                           float alt)
{
    std::vector<uint8_t> packet(PACKET_SIZE, 0);

    packet[0] = PACKET_START_BYTE;
    packet[1] = drone_id;
    packet[2] = PKT_TYPE_WAYPOINT;

    // Copy 3 floats into payload starting at byte 3
    float coords[3] = { lat, lon, alt };
    std::memcpy(&packet[3], coords, sizeof(coords));

    packet[PACKET_SIZE - 1] = PACKET_END_BYTE;
    return packet;
}

bool send_packet_fd(int fd, const std::vector<uint8_t>& packet)
{
    if (fd < 0 || packet.size() != PACKET_SIZE) return false;

    const uint8_t* data = packet.data();
    size_t remaining = packet.size();

    while (remaining > 0) {
        ssize_t n = ::write(fd, data, remaining);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        data += n;
        remaining -= static_cast<size_t>(n);
    }
    return true;
}

bool send_waypoint_fd(int fd, uint8_t drone_id, double lat, double lon, double alt)
{
    auto packet = build_waypoint_packet(
        drone_id,
        static_cast<float>(lat),
        static_cast<float>(lon),
        static_cast<float>(alt)
    );
    return send_packet_fd(fd, packet);
}

void send_locations_fd(int fd,
                       uint8_t drone_id,
                       const std::vector<Location>& locations,
                       float alt = 50.0f)
{
    for (const auto& loc : locations) {
        if (!send_waypoint_fd(fd, drone_id, loc.lat, loc.lon, alt)) {
            std::cerr << "Failed sending waypoint for drone "
                      << static_cast<int>(drone_id) << "\n";
            return;
        }
        usleep(50000); // 50 ms gap; adjust as needed
    }
}