#ifndef _MC_STRUCTS_H
#define _MC_STRUCTS_H

#include <stdint.h>

namespace minicrusher_base {

    /* Standard wheel order for any 6 element arrays in these packets is below.
     * Numbers are the index into the array for that wheel
     *
     *     Front
     * Left    Right
     *  0        1
     *  2        3
     *  4        5
     *
     */

    enum packet_types { PACKET_CMD = 0, PACKET_STATUS, PACKET_ENCODER };

    static const unsigned int MAX_PAYLOAD_SIZE = 42 - 4; //max packet length - 4 header bytes for max size of payload

    static const unsigned int CMD_PACKET_SIZE = 19; //includes 4 header bytes
    typedef struct {
        int16_t wheel_vels_mmps[6]; //wheel velocities in mm/s in standard wheel order
        uint8_t enable; // valid values are 1 or 0 (enable/disable respectively)
    } command_packet_t;

    static const unsigned int ENCODER_PACKET_SIZE = 30; //includes 4 header bytes
    typedef struct {
        int32_t positions_ticks[6]; //encoder positions for each wheel in ticks and in standard wheel order
    } encoder_packet_t;

    static const unsigned int STATUS_PACKET_SIZE = 42; //includes 4 header bytes
    typedef struct {
        uint8_t motor_temps_deg_f[6]; // motor (windings) temps in degrees Fahrenheit in standard wheel order
        uint8_t drive_temps_deg_f[6]; // drive (mosfets) temps in degrees Fahrenheit in standard wheel order
        uint8_t swing_arm_angles_ticks[6]; // ticks (1024 counts per revolution; undefined starting point).
        int16_t current_mA[6]; // standard wheel order as defined above
        int16_t battery_current_mA; // battery current in milliamps
        int16_t battery_voltage_cV; // battery voltage x 100 (so centivolts)
        int16_t time; // microseconds since powerup. May or may not be populated
    } status_packet_t;

    typedef struct {
        uint8_t sync1; // should be "M" for a valid packet
        uint8_t sync2; // should be "C" for a valid packet
        uint8_t type; // the packet type, should be one of packet_types
        uint8_t length; // length in bytes of the packet. Includes these header bytes and length(data) for this packet type
        union {
            uint8_t payload[MAX_PAYLOAD_SIZE]; // last two bytes of length(data) are the checksum bytes
            command_packet_t command_packet;
            encoder_packet_t encoder_packet;
            status_packet_t status_packet;
        };
    } mc_packet_t;

}; //namespace minicrusher_base

#endif //#ifndef _MC_STRUCTS_H
