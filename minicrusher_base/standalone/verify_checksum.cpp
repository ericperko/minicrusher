#include <minicrusher_base/mc_structs.h>
#include <boost/crc.hpp>
#include <iostream>
#include <string.h>
//#include "fcs.h"

typedef boost::crc_optimal<16, 0x1021, 0xFFFF, 0x0, true, true> mc_crc_t;
static const uint16_t good_checksum = 0xF0B8;

using namespace minicrusher_base;

int main()
{

   uint8_t payload[26] = {0x00, 0x00, 0x0a, 0x7f, 0xff, 0xff, 0xed, 0x15, 0xff, 0xff, 0xff, 0x41, 0xff, 0xff, 0xf8, 0x51, 0xff, 0xff, 0xef, 0xae, 0xff, 0xff, 0xe9, 0xe0, 0xf1, 0x29};

   mc_packet_t sample_packet;
   sample_packet.sync1 = 'M';
   sample_packet.sync2 = 'C';
   sample_packet.type = 0x02;
   sample_packet.length = 0x1e;
   memcpy(sample_packet.payload, payload, 26);
   mc_crc_t crc_calc, crc_calc2, crc_calc3, crc_calc4;
   crc_calc.process_bytes(&sample_packet, sample_packet.length);
   std::cout << "Good checksum: " << (int) good_checksum << std::endl;
   std::cout << "First way: " << (int) crc_calc.checksum() << std::endl;

   crc_calc2.process_byte(sample_packet.sync1);
   crc_calc2.process_byte(sample_packet.sync2);
   crc_calc2.process_byte(sample_packet.type);
   crc_calc2.process_byte(sample_packet.length);
//   crc_calc2.process_bytes(sample_packet.payload, 26);
   for(uint8_t i = 0; i < 26; i++) 
   {
	crc_calc2.process_byte(sample_packet.payload[i]);
   }
   std::cout << "Second way: " << (int) crc_calc2.checksum() << std::endl;

   crc_calc3.process_bytes((unsigned char*) &sample_packet, 30);
   std::cout << "Third way: " << (int) crc_calc3.checksum() << std::endl;

/*   unsigned short fcs = fcs16(0xFFFF, (unsigned char*) &sample_packet, 30);
   std::cout << "FCS16 " << fcs << std::endl;

   crc_calc4.process_bytes(payload, 26);
   std::cout << "Payload only. Boost, then FCS: " << (int) crc_calc4.checksum() << " " << fcs16(0xFFFF, payload, 26) << std::endl;
*/
   /*std::cout << "Searching for the correct parameters..." << std::endl;
   for(uint16_t i = 0x0000; i < 0xFFFF; i++)
   {
      boost::crc_basic<16> crc(0x1021, 0xFFFF, 0x0, true, true);
      crc.process_bytes(&sample_packet, 30);
      if(crc.checksum() == good_checksum)
      {
	std::cout << "Found: " << i << std::endl;
        break;
      }
   }*/

   std::cout << "Checking the add FCS crap...";
   crc_calc4.reset();
   crc_calc4.process_bytes(&sample_packet, 28);
   std::cout << "Computed checksum was: " << (int) crc_calc4.checksum() << std::endl;
   uint16_t checksum = crc_calc4.checksum();
   checksum ^= 0xFFFF;
   std::cout << "Monkeyed checksum was: " << (int) checksum
 	     << " First: " << (int) (checksum & 0x00FF) 
 	     << " Second: " << (int) ((checksum >> 8) & 0x00FF) << std::endl;

   std::cout << "Original payload checksum was: First: " << (int) 0xf1 << " Second: " << (int) 0x29 << std::endl;


}
