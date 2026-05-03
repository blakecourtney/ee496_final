import struct

PACKET_START_BYTE  = 0xFE
PACKET_END_BYTE    = 0xFF

PKT_TYPE_TELEMETRY   = 1
PKT_TYPE_COMMAND     = 2
PKT_TYPE_FLAG        = 3
PKT_TYPE_HEARTBEAT   = 4
PKT_TYPE_WAYPOINT    = 5
PKT_TYPE_ACK         = 7
PKT_TYPE_FLAG_ACK    = 8
PKT_TYPE_PHOTO_CHUNK = 9
PKT_TYPE_PHOTO_DONE  = 10

PACKET_SIZE       = 68   # sizeof(packet_t):  1+1+1+64+1
PHOTO_PACKET_SIZE = 242  # sizeof(photo_packet_t): 1+1+1+1pad+2+2+2+230+1+1pad
PHOTO_CHUNK_SIZE  = 400
PHOTO_PACKET_SIZE = 411

class TelemetryParser:
    @staticmethod
    def parse(data):
        if len(data) < 3:
            return None
        if data[0] != PACKET_START_BYTE:
            return None

        drone_id    = data[1]
        packet_type = data[2]

        if packet_type == PKT_TYPE_PHOTO_CHUNK:
            return TelemetryParser._parse_photo_chunk(drone_id, data)

        # All other types use the standard 68-byte packet_t
        if len(data) < PACKET_SIZE:
            return None
        if data[PACKET_SIZE - 1] != PACKET_END_BYTE:
            return None

        payload = data[3:67]  # 64 bytes

        if packet_type == PKT_TYPE_TELEMETRY:
            return TelemetryParser._parse_telemetry(drone_id, payload)
        elif packet_type == PKT_TYPE_HEARTBEAT:
            return TelemetryParser._parse_heartbeat(drone_id, payload)
        elif packet_type == PKT_TYPE_FLAG:
            return TelemetryParser._parse_flag(drone_id, payload)
        elif packet_type == PKT_TYPE_PHOTO_DONE:
            return {'type': 'photo_done', 'id': drone_id}

        return None

    @staticmethod
    def _parse_photo_chunk(drone_id, data):
        # photo_packet_t layout (with 1-byte compiler padding after `type`):
        #   [0]      start        (0xFE)
        #   [1]      drone_id
        #   [2]      type         (9)
        #   [3]      pad          (alignment byte — uint16_t needs 2-byte align)
        #   [4:6]    chunk_index  uint16_t LE
        #   [6:8]    total_chunks uint16_t LE
        #   [8:10]   data_len     uint16_t LE
        #   [10:240] data         230 bytes
        #   [240]    end          (0xFF)
        #   [241]    trailing pad (sizeof = 242)
        print(f"[chunk] received {len(data)} bytes, last_byte={hex(data[-1]) if data else 'none'}, end@240={hex(data[240]) if len(data)>240 else 'n/a'}")
        print(list(data[:20]))
        if len(data) < PHOTO_PACKET_SIZE:
            return None
        # if data[240] != PACKET_END_BYTE:
        #     return None
        try:
            chunk_index, total_chunks, data_len = struct.unpack_from('<HHH', data, 4)
            chunk_data = bytes(data[10:10 + data_len])
            return {
                'type':         'photo_chunk',
                'id':           drone_id,
                'chunk_index':  chunk_index,
                'total_chunks': total_chunks,
                'data':         chunk_data,
            }
        except Exception as e:
            print(f"Photo chunk parse error: {e}")
            return None

    @staticmethod
    def _parse_telemetry(drone_id, payload):
        try:
            lat, lon, alt, roll, pitch, yaw, battery = struct.unpack_from('7f', payload, 0)
            satellites = payload[28]
            armed      = bool(payload[29])
            return {
                'type':       'telemetry',
                'id':         drone_id,
                'lat':        lat,
                'lon':        lon,
                'alt':        alt,
                'roll':       roll,
                'pitch':      pitch,
                'yaw':        yaw,
                'battery':    battery,
                'satellites': satellites,
                'armed':      armed,
                'streaming':  False,
            }
        except Exception as e:
            print(f"Telemetry parse error: {e}")
            return None

    @staticmethod
    def _parse_heartbeat(drone_id, _):
        return {
            'type':  'heartbeat',
            'id':    drone_id,
            'armed': False,
        }

    @staticmethod
    def _parse_flag(drone_id, payload):
        try:
            lat, lon, alt, confidence = struct.unpack_from('4f', payload, 0)
            return {
                'type':       'flag',
                'id':         drone_id,
                'lat':        lat,
                'lon':        lon,
                'alt':        alt,
                'confidence': confidence,
            }
        except Exception as e:
            print(f"Flag parse error: {e}")
            return None
