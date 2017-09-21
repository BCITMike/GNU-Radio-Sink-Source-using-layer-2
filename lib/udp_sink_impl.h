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

#ifndef INCLUDED_GRSBT_udp_sink_impl_H
#define INCLUDED_GRSBT_udp_sink_impl_H

#include <grsbt/udp_sink.h>
#include <boost/asio.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/copy.hpp>

#include "udpHeaderTypes.h"

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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>

namespace gr {
  namespace grsbt {

    class GRSBT_API udp_sink_impl : public udp_sink
    {
     private:
        size_t d_itemsize;
        size_t d_veclen;
        size_t d_block_size;

        int d_header_type;
        int d_header_size;
        unsigned int d_seq_num;
        
        const std::string d_send_iface;
        const std::string d_destMac;

        int d_payload_size;    // maximum transmission unit (packet length)
        bool d_eof;             // send zero-length packet on disconnect
        bool d_connected;       // are we connected?    

        char tmpHeaderBuff[12];  // 32-bit sync word (0xFFFFFFFF), 32-bit sequence num and 32-bit data size

        unsigned char custom_type[2] = {0x33,0x33};
        unsigned char ethHeader[12] = {0x00,0x0c,0x29,0x05,0x0d,0x15,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        unsigned char mac_str[6];

        unsigned char dstMAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};    
        unsigned char srcMAC[6];
        
        boost::system::error_code ec;
/* 1G management stuff */
        boost::asio::io_service d_io_service;
        boost::asio::ip::udp::endpoint d_endpoint;
        boost::asio::ip::udp::socket *udpsocket;
/* 1G management stuff */

/* 10G raw socket stuff */    
      typedef boost::asio::generic::raw_protocol raw_protocol_t;
      typedef boost::asio::generic::basic_endpoint<raw_protocol_t> raw_endpoint_t;      
      boost::asio::io_service d_io_service10;
      
      raw_protocol_t::socket *d_socket10;
/* 10G raw socket stuff */

        boost::mutex d_mutex;

     public:
      udp_sink_impl(size_t itemsize, size_t vecLen, const std::string &host, int port, int headerType, int payload_size, bool eof, const std::string &send_iface, const std::string &destMac);
      
      ~udp_sink_impl();

     bool stop();

     
     void getMACAddress(std::string _iface,unsigned char MAC[6]);
   

      // Where all the action really happens
      int work_test(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace grsbt
} // namespace gr

#endif /* INCLUDED_GRSBT_udp_sink_impl_H */

