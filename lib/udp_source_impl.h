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

#ifndef INCLUDED_GRSBT_udp_source_impl_H
#define INCLUDED_GRSBT_udp_source_impl_H

#include <grsbt/udp_source.h>
#include <boost/asio.hpp>
#include <boost/asio/ip/udp.hpp>
#include <queue>

#include "udpHeaderTypes.h"

namespace gr {
  namespace grsbt {

    class udp_source_impl : public udp_source
    {
     private:
        size_t d_itemsize;
        size_t d_veclen;
        size_t d_block_size;
        bool d_eof;
        bool d_connected;
        int d_payload_size;
        char   *d_rxbuf;        // get UDP buffer items
        char   *d_residbuf;     // hold buffer between calls
        ssize_t d_residual;     // hold information about number of bytes stored in residbuf
        ssize_t d_sent;         // track how much of d_residbuf we've outputted
        size_t  d_offset;       // point to residbuf location offset
        
        const std::string d_receive_iface; 
        const std::string d_sendMac;
        const std::string d_mgmt;

        boost::system::error_code ec;

        boost::asio::io_service d_io_service;
        boost::asio::ip::udp::endpoint d_endpoint;
        boost::asio::ip::udp::socket *udpsocket;

        boost::asio::streambuf read_buffer;
    	std::queue<char> localQueue;

        boost::mutex d_mutex;

    	int maxSize;
     public:
      udp_source_impl(size_t itemsize,size_t vecLen, int port, int payload_size, bool eof, const std::string &receive_iface, const std::string &sendMac, const std::string &mgmt_addr);
      ~udp_source_impl();

      bool stop();

      size_t dataAvailable();
      size_t netDataAvailable();

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

#endif /* INCLUDED_GRSBT_udp_source_impl_H */

