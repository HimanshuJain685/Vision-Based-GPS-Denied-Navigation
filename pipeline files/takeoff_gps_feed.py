import socket
import json
import time
#------------------------------------------------------------------------------------------------------#
# Configuration Variables
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPV4, UDP
out_addr = ("127.0.0.1", 25100)

# Loop to feed constant GPS data
while True:
    time.sleep(0.05)
    data = {
            'time_usec' : 0,                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
            'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
            'ignore_flags' : 12,                    # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
            'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
            'time_week' : 0,                        # (uint16_t) GPS week number
            'fix_type' : 3,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
            'lat' : 189923947,                      # (int32_t) Latitude (WGS84), in degrees * 1E7
            'lon' : 728352990,                      # (int32_t) Longitude (WGS84), in degrees * 1E7
            'alt' : 0,                              # (float) Altitude (AMSL, not WGS84), in m (positive for up)
            'hdop' : 1,                             # (float) GPS HDOP horizontal dilution of position in m
            'vdop' : 1,                             # (float) GPS VDOP vertical dilution of position in m
            'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
            've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
            'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
            'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
            'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
            'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
            'satellites_visible' : 13               # (uint8_t) Number of satellites visible.
    }

    out_data = json.dumps(data)
    print('out:',out_data)
    s.sendto(out_data.encode(), out_addr)