/* -*- c++ -*- */
/* 
 * Copyright 2017 Silver Bullet Technologies.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "udp_sink_impl.h"
#include <zlib.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <gnuradio/thread/thread.h>
#include <stdexcept>
#include <stdio.h>
#include <string.h>

#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <net/if.h>

#include <iostream>
#include <iterator>
#include <string>
#include <deque>
#include <stdexcept>
#include <cstdint>
#include <utility>

#include <iostream>
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <iomanip>

namespace gr {
  namespace grsbt {

    udp_sink::sptr
    udp_sink::make(size_t itemsize,size_t vecLen,const std::string &host, int port,int headerType, int payload_size, bool eof, const std::string &send_iface, const std::string &destMac)
    {
      return gnuradio::get_initial_sptr
        (new udp_sink_impl(itemsize, vecLen,host, port,headerType,payload_size,eof,send_iface,destMac));
    }

    /*
     * The private constructor
     */
    udp_sink_impl::udp_sink_impl(size_t itemsize,size_t vecLen,const std::string &host, int port,int headerType, int payload_size, bool eof, const std::string &send_iface, const std::string &destMac)
      : gr::sync_block("udp_sink",
              gr::io_signature::make(1, 1, (itemsize*vecLen)),
              gr::io_signature::make(0, 0, 0)),
    d_itemsize(itemsize), d_veclen(vecLen), d_header_type(headerType), d_seq_num(0), d_header_size(0), d_payload_size(payload_size), d_eof(eof), d_send_iface(send_iface), d_destMac(destMac)
    {
        
      d_payload_size=d_payload_size-16;  

      switch (d_header_type) {
        	case HEADERTYPE_NONE:
    		    d_header_size = 0;
        	case HEADERTYPE_SEQNUM:
        		d_payload_size = d_payload_size - 8;  // take back our header
            d_header_size = 8;
        	break;

        	case HEADERTYPE_SEQPLUSSIZE:
        		d_payload_size = d_payload_size - 12; // take back our header
            d_header_size = 12;
        	break;

        	case HEADERTYPE_SEQSIZECRC:
        		d_payload_size = d_payload_size - 12 - sizeof(unsigned long); // Take back header and trailing crc
            d_header_size = 12;
        	break;
        }
      
    
      d_block_size = d_itemsize * d_veclen;

      getMACAddress(d_send_iface,srcMAC);

/* Format Destination MAC string */      
      unsigned int value;
      char ignore;
    
      std::istringstream iss(d_destMac,std::istringstream::in);
    
      iss >> std::hex;
    
      for(int p=0;p<5;p++) {
        iss >> value >> ignore;
        dstMAC[p]=value;
      }
      iss >> value;
      dstMAC[5]=value;
    
      // validate
      std::cout << "Destination MAC: ";    
      for(int p=0;p<sizeof(dstMAC)/sizeof(dstMAC[0]);p++)
      std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << static_cast<unsigned int>(dstMAC[p]) << " ";
    
      std::cout << std::endl;
         
/* end destination MAC */      
      
      
/* Begin 1G management connection */
      std::string s__port = (boost::format("%d") % port).str();
      std::string s__host = host.empty() ? std::string("localhost") : host;
      boost::asio::ip::udp::resolver resolver(d_io_service);
      boost::asio::ip::udp::resolver::query query(s__host, s__port,
          boost::asio::ip::resolver_query_base::passive);
      d_endpoint = *resolver.resolve(query);

		  udpsocket = new boost::asio::ip::udp::socket(d_io_service);
    	udpsocket->connect(d_endpoint);
/* End 1G management connection */
/* Begin 10G management connection */      
      sockaddr_ll sockaddr;
      memset(&sockaddr, 0, sizeof(sockaddr));
      sockaddr.sll_family = PF_PACKET;
      sockaddr.sll_protocol = htons(ETH_P_ALL);
      sockaddr.sll_ifindex = if_nametoindex(send_iface.c_str());
      sockaddr.sll_hatype = 1;
  
      raw_protocol_t::socket d_socket10(d_io_service10, raw_protocol_t(PF_PACKET, SOCK_RAW));   /* Needs root or cap_net_raw permissions */   

      d_socket10.bind(raw_endpoint_t(&sockaddr, sizeof(sockaddr)));
/* End 10G management connection */

      /* Just for testing */
      std::vector<boost::asio::const_buffer> transmitbuffer2;        
      transmitbuffer2.clear();
      transmitbuffer2.push_back(boost::asio::buffer((const void *)dstMAC, sizeof(dstMAC)));
      transmitbuffer2.push_back(boost::asio::buffer((const void *)srcMAC, sizeof(srcMAC)));
      transmitbuffer2.push_back(boost::asio::buffer((const void *)custom_type, sizeof(custom_type)));
      d_socket10.send(transmitbuffer2);


    boost::asio::streambuf buffer;
    std::ostream stream( &buffer );    
    unsigned char const deadbeef[5] = { 0x00, 0xde, 0xad, 0xbe, 0xef };
    stream << "Hello, World!!!";
    stream.write(reinterpret_cast<const char*>(&deadbeef[0]), 5); 
    d_socket10.send(buffer.data());    

    } // Close constructor

    /*
     * Our virtual destructor.
     */
    udp_sink_impl::~udp_sink_impl()
    {
    	stop();
    }

    bool 
    udp_sink_impl::stop() {
        if (udpsocket) {
          gr::thread::scoped_lock guard(d_mutex);  // protect d_socket from work()

          // Send a few zero-length packets to signal receiver we are done
          
          if(d_eof) {
            boost::array<char, 0> send_buf;
            for(int i = 0; i < 3; i++)
              udpsocket->send_to(boost::asio::buffer(send_buf), d_endpoint); 
          }
        	udpsocket->close();

        	udpsocket = NULL;

            d_io_service.reset();
            d_io_service.stop();
        }
        return true;
    }
  
    void 
    udp_sink_impl::getMACAddress(std::string _iface,unsigned char MAC[6]) {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        struct ifreq ifr;
        ifr.ifr_addr.sa_family = AF_INET;
        strncpy(ifr.ifr_name , _iface.c_str() , IFNAMSIZ-1);
        ioctl(fd, SIOCGIFHWADDR, &ifr);
        for(unsigned int i=0;i<6;i++)
            MAC[i] = ifr.ifr_hwaddr.sa_data[i];
        ioctl(fd, SIOCGIFMTU, &ifr);
        close(fd);
        printf("MTU: %d\n",ifr.ifr_mtu);
        printf("MAC:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",MAC[0],MAC[1],MAC[2],MAC[3],MAC[4],MAC[5]);
        
        if ( ifr.ifr_mtu < d_payload_size ) {
          std::cout << "WARNING! MTU IS LESS THAN PAYLOAD AND WILL LIKELY HAVE PROBLEMS. GO BACK AND INCREASE MTU FOR NIC [" << d_send_iface << "] or decrease payload size\n" << "MTU:" << ifr.ifr_mtu << " PAYLOAD:" << d_payload_size << std::endl;
      }
    }    
    
    int
    udp_sink_impl::work_test(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
        gr::thread::scoped_lock guard(d_mutex);

      const char *in = (const char *) input_items[0];
    	unsigned int noi = noutput_items * d_block_size;

    	// Calc our packet break-up
    	float fNumPackets = (float)noi / (float)d_payload_size;
    	int numPackets = noi / d_payload_size;
    	if (fNumPackets > (float)numPackets)
    		numPackets = numPackets + 1;

    	// deal with a last packet < d_payload_size
    	int lastPacketSize = noi - (noi / d_payload_size) * d_payload_size;

    	std::vector<boost::asio::const_buffer> transmitbuffer;
    	int curPtr=0;

    	for (int curPacket=0;curPacket < numPackets; curPacket++) {
    		// Clear the next transmit buffer
    		transmitbuffer.clear();
        
        // Generate ethernet header
        transmitbuffer.push_back(boost::asio::buffer((const void *)srcMAC, sizeof(srcMAC)));   
        transmitbuffer.push_back(boost::asio::buffer((const void *)dstMAC, sizeof(dstMAC)));
       
        // Generate custom data type
        transmitbuffer.push_back(boost::asio::buffer((const void *)custom_type, sizeof(custom_type)));
      
    		// build our next header if we need it
            if (d_header_type != HEADERTYPE_NONE) {
            	if (d_seq_num == 0xFFFFFFFF)
            		d_seq_num = 0;

            	d_seq_num++;
            	// want to send the header.
            	tmpHeaderBuff[0]=tmpHeaderBuff[1]=tmpHeaderBuff[2]=tmpHeaderBuff[3]=0xFF;

                memcpy((void *)&tmpHeaderBuff[4], (void *)&d_seq_num, sizeof(d_seq_num));

                if ((d_header_type == HEADERTYPE_SEQPLUSSIZE)||(d_header_type == HEADERTYPE_SEQSIZECRC)) {
                    if (curPacket < (numPackets-1))
                        memcpy((void *)&tmpHeaderBuff[8], (void *)&d_payload_size, sizeof(d_payload_size));
                    else
                        memcpy((void *)&tmpHeaderBuff[8], (void *)&lastPacketSize, sizeof(lastPacketSize));
                }

                transmitbuffer.push_back(boost::asio::buffer((const void *)tmpHeaderBuff, d_header_size));

            }

            // Grab our next data chunk
            if (curPacket < (numPackets-1)) {
            	transmitbuffer.push_back(boost::asio::buffer((const void *)&in[curPtr], d_payload_size));
            }
            else {
            	transmitbuffer.push_back(boost::asio::buffer((const void *)&in[curPtr], lastPacketSize));
            }

            if (d_header_type == HEADERTYPE_SEQSIZECRC) {
            	unsigned long  crc = crc32(0L, Z_NULL, 0);
                if (curPacket < (numPackets-1))
                	crc = crc32(crc, (const unsigned char*)&in[curPtr], d_payload_size);
                else
                	crc = crc32(crc, (const unsigned char*)&in[curPtr], lastPacketSize);

                memcpy((void *)tmpHeaderBuff, (void *)&crc, sizeof(crc));
                transmitbuffer.push_back(boost::asio::buffer((const void *)tmpHeaderBuff, sizeof(crc)));
            }

             udpsocket->send_to(transmitbuffer,d_endpoint);


            curPtr = curPtr + d_payload_size;
    	}

        return noutput_items;
    }

    int
    udp_sink_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
        gr::thread::scoped_lock guard(d_mutex);

      const char *in = (const char *) input_items[0];
    	unsigned int noi = noutput_items * d_block_size;

    	// Calc our packet break-up
    	float fNumPackets = (float)noi / (float)d_payload_size;
    	int numPackets = noi / d_payload_size;
    	if (fNumPackets > (float)numPackets)
    		numPackets = numPackets + 1;

    	// deal with a last packet < d_payload_size
    	int lastPacketSize = noi - (noi / d_payload_size) * d_payload_size;

    	std::vector<boost::asio::const_buffer> transmitbuffer;
    	int curPtr=0;

    	for (int curPacket=0;curPacket < numPackets; curPacket++) {
    		// Clear the next transmit buffer
    		transmitbuffer.clear();
                
        // Generate ethernet header
        transmitbuffer.push_back(boost::asio::buffer((const void *)srcMAC, sizeof(srcMAC)));   
        transmitbuffer.push_back(boost::asio::buffer((const void *)dstMAC, sizeof(dstMAC)));
       
        // Generate custom data type
        transmitbuffer.push_back(boost::asio::buffer((const void *)custom_type, sizeof(custom_type)));
     
    		// build our next header if we need it
            if (d_header_type != HEADERTYPE_NONE) {
            	if (d_seq_num == 0xFFFFFFFF)
            		d_seq_num = 0;

            	d_seq_num++;
            	// want to send the header.
            	tmpHeaderBuff[0]=tmpHeaderBuff[1]=tmpHeaderBuff[2]=tmpHeaderBuff[3]=0xFF;

                memcpy((void *)&tmpHeaderBuff[4], (void *)&d_seq_num, sizeof(d_seq_num));

                if ((d_header_type == HEADERTYPE_SEQPLUSSIZE)||(d_header_type == HEADERTYPE_SEQSIZECRC)) {
                    if (curPacket < (numPackets-1))
                        memcpy((void *)&tmpHeaderBuff[8], (void *)&d_payload_size, sizeof(d_payload_size));
                    else
                        memcpy((void *)&tmpHeaderBuff[8], (void *)&lastPacketSize, sizeof(lastPacketSize));
                }

                transmitbuffer.push_back(boost::asio::buffer((const void *)tmpHeaderBuff, d_header_size));

            }

            // Grab our next data chunk
            if (curPacket < (numPackets-1)) {
            	transmitbuffer.push_back(boost::asio::buffer((const void *)&in[curPtr], d_payload_size));
            }
            else {
            	transmitbuffer.push_back(boost::asio::buffer((const void *)&in[curPtr], lastPacketSize));
            }

            if (d_header_type == HEADERTYPE_SEQSIZECRC) {
            	unsigned long  crc = crc32(0L, Z_NULL, 0);
                if (curPacket < (numPackets-1))
                	crc = crc32(crc, (const unsigned char*)&in[curPtr], d_payload_size);
                else
                	crc = crc32(crc, (const unsigned char*)&in[curPtr], lastPacketSize);

                memcpy((void *)tmpHeaderBuff, (void *)&crc, sizeof(crc));
                transmitbuffer.push_back(boost::asio::buffer((const void *)tmpHeaderBuff, sizeof(crc)));
            }

            udpsocket->send_to(transmitbuffer,d_endpoint);
            //d_socket10.send(transmitbuffer);
            //d_socket10->send(transmitbuffer);
        
         

            curPtr = curPtr + d_payload_size;
    	}

        return noutput_items;
    }
  } /* namespace grsbt */
} /* namespace gr */

