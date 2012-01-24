#ifndef _MC_STRUCTS_H
#define _MC_STRUCTS_H

#include <stdint.h>

namespace minicrusher_base {

    enum packet_types { PACKET_CMD = 0, PACKET_STATUS, PACKET_ENCODER }; //so what's the radio packet type?

    static const unsigned int MAX_PAYLOAD_SIZE = 42 - 4; //max packet length - 4 header bytes for max size of payload

    typedef struct {
        uint8_t sync1; // should be "M" for a valid packet
        uint8_t sync2; // should be "C" for a valid packet
        uint8_t type; // the packet type, should be one of packet_types
        uint8_t length; // length in bytes of the packet. Includes these header bytes and length(data) for this packet type
        uint8_t payload[MAX_PAYLOAD_SIZE]; // last two bytes of length(data) are the checksum bytes
    } mc_packet_t;

    typedef struct {
        int16_t wheel_vels_mmps[6]; //wheel velocities in mm/s (what order are wheels in??)
        uint8_t enable; // valid values are 1 or 0?? 1 means enable motors? 0 means disable??
        uint8_t timeout; // ?? units?? when the timeout expires, send 0's to wheels??
    } command_packet_t;

   typedef struct {
       int32_t positions[6]; //encoder positions for each wheel (what units?? ticks??) (order of wheels??)
       uint8_t fresh; //valid values?? how should I interpret the "fresh" value? been updated since the last packet was sent? is this even used?
   } encoder_packet_t;

   typedef struct {
       int16_t sticks[6]; //position of each radio stick?? (order of sticks???)
       uint8_t fresh; //valid values??
   } radio_packet_t;

   typedef struct {
       uint8_t motor_temps_deg_f[6]; // motor temps in degrees Fahrenheit (order of wheels??)
       uint8_t drive_temps_deg_f[6]; // drive temps in degrees Fahrenheit (order of wheels???) (how is different than motor temps??)
       uint8_t angle_sensor_units[6]; // order of sensors?? units?? what angle is being sensed?
       int16_t current_mA[6]; // order of sensors??
       int16_t battery_current_units; //units?
       int16_t battery_voltage_units; // battery voltage x 100 (so centivolts???)
       int16_t time; //relative to what? units???
       uint8_t fresh; //valid values??
   } status_packet_t;

}; //namespace minicrusher_base

#endif //#ifndef _MC_STRUCTS_H
