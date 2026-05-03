# build CMD packets to send to drones
import struct

PACKET_START_BYTE = 0xFE
PACKET_END_BYTE   = 0xFF
PKT_TYPE_COMMAND  = 2
PKT_TYPE_WAYPOINT = 5
PKT_TYPE_FLAG_ACK = 8

PAYLOAD_SIZE = 64

CMD_ARM         = 1
CMD_DISARM      = 0
CMD_START_VIDEO = 20
CMD_STOP_VIDEO  = 21

class CommandBuilder:
    @staticmethod
    def build(drone_id, packet_type, payload=None):
        """Build packet as bytes matching packet_t struct"""
        packet = bytearray()
        packet.append(PACKET_START_BYTE)
        packet.append(drone_id)
        packet.append(packet_type)

        # pad payload to exactly 64 bytes
        if payload:
            padded = bytearray(payload)
            padded.extend([0] * (PAYLOAD_SIZE - len(padded)))
        else:
            padded = bytearray(PAYLOAD_SIZE)

        packet.extend(padded)
        packet.append(PACKET_END_BYTE)
        return bytes(packet)

    @staticmethod
    def arm(drone_id):
        payload = bytearray([CMD_ARM])
        return CommandBuilder.build(drone_id, PKT_TYPE_COMMAND, payload)

    @staticmethod
    def disarm(drone_id):
        payload = bytearray([CMD_DISARM])
        return CommandBuilder.build(drone_id, PKT_TYPE_COMMAND, payload)

    @staticmethod
    def flag_ack(drone_id):
        return CommandBuilder.build(drone_id, PKT_TYPE_FLAG_ACK)

    @staticmethod
    def waypoint(drone_id, lat, lon, alt):
        """Send waypoint coordinates to drone"""
        payload = struct.pack('3f', lat, lon, alt)
        return CommandBuilder.build(drone_id, PKT_TYPE_WAYPOINT, payload)