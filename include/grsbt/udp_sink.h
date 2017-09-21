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


#ifndef INCLUDED_GRSBT_UDP_SINK_H
#define INCLUDED_GRSBT_UDP_SINK_H

#include <grsbt/api.h>
#include <gnuradio/sync_block.h>
#include "udpHeaderTypes.h"
//#include <boost/asio.hpp>

namespace gr {
  namespace grsbt {

    /*!
     * \brief <+description of block+>
     * \ingroup grsbt
     *
     */
    class GRSBT_API udp_sink : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<udp_sink> sptr;
      /*!
       * \brief Return a shared_ptr to a new instance of grsbt::udp_sink.
       *
       * To avoid accidental use of raw pointers, grsbt::udp_sink's
       * constructor is in a private implementation
       * class. grsbt::udp_sink::make is the public interface for
       * creating new instances.
       */
      //static sptr make(size_t itemsize,size_t vecLen, const std::string &host, int port,int headerType);
      static sptr make(size_t itemsize,size_t vecLen, const std::string &host, int port, int headerType=HEADERTYPE_NONE, int payload_size=1472, bool eof=true, const std::string &send_iface="eth0", const std::string &destMac="FF:FF:FF:FF:FF:FF");
      
      virtual bool stop() = 0;
      virtual void getMACAddress(std::string _iface,unsigned char MAC[6]) = 0;
    };

  } // namespace grsbt
} // namespace gr

#endif /* INCLUDED_GRSBT_UDP_SINK_H */

