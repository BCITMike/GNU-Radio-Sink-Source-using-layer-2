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
#include "udp_source_impl.h"

namespace gr {
  namespace grsbt {

    udp_source::sptr
    udp_source::make(size_t itemsize,size_t vecLen,int port, int payload_size, bool eof, const std::string &receive_iface, const std::string &sendMac, const std::string &mgmt_addr)
    {
      return gnuradio::get_initial_sptr
        (new udp_source_impl(itemsize, vecLen,port,payload_size,eof,receive_iface,sendMac,mgmt_addr));
    }

    /*
     * The private constructor
     */
    udp_source_impl::udp_source_impl(size_t itemsize,size_t vecLen,int port, int payload_size, bool eof, const std::string &receive_iface, const std::string &sendMac,const std::string &mgmt_addr)
      : gr::sync_block("udp_source",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, itemsize*vecLen)),
    d_itemsize(itemsize), d_veclen(vecLen), d_eof(eof), d_receive_iface(receive_iface), d_sendMac(sendMac), d_payload_size(payload_size)
    {
    	maxSize=256*1024;

    	d_block_size = d_itemsize * d_veclen;

    	/*
        std::string s__port = (boost::format("%d") % port).str();
        std::string s__host = "0.0.0.0";
        boost::asio::ip::udp::resolver resolver(d_io_service);
        boost::asio::ip::udp::resolver::query query(s__host, s__port,
            boost::asio::ip::resolver_query_base::passive);
        */
        d_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port);

		udpsocket = new boost::asio::ip::udp::socket(d_io_service,d_endpoint);

//      std::cout << "\nreceive_iface: " << d_receive_iface;
      std::cout << "\nsendMac: " << d_sendMac;
      std::cout << "\nPayload size: " << d_payload_size << std::endl;

      unsigned char MAC[6];
      int fd = socket(AF_INET, SOCK_DGRAM, 0);
      struct ifreq ifr;
      ifr.ifr_addr.sa_family = AF_INET;
      strncpy(ifr.ifr_name , d_receive_iface.c_str() , IFNAMSIZ-1);
      ioctl(fd, SIOCGIFHWADDR, &ifr);
      for(unsigned int i=0;i<6;i++)
          MAC[i] = ifr.ifr_hwaddr.sa_data[i];
      ioctl(fd, SIOCGIFMTU, &ifr);
      close(fd);
      std::cout << "Interface: " << d_receive_iface;
      printf("\nMTU: %d\n",ifr.ifr_mtu);
      printf("MAC:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",MAC[0],MAC[1],MAC[2],MAC[3],MAC[4],MAC[5]);
      
      if ( ifr.ifr_mtu < d_payload_size ) {
        std::cout << "WARNING! MTU IS LESS THAN PAYLOAD AND WILL LIKELY HAVE PROBLEMS. GO BACK AND INCREASE MTU FOR NIC [" << d_receive_iface << "] or decrease payload size\n" << "MTU:" << ifr.ifr_mtu << " PAYLOAD:" << d_payload_size << std::endl;
      }
      
    }

    /*
     * Our virtual destructor.
     */
    udp_source_impl::~udp_source_impl()
    {
    	stop();
    }

    bool udp_source_impl::stop() {
        if (udpsocket) {
			udpsocket->close();

			udpsocket = NULL;

            d_io_service.reset();
            d_io_service.stop();
        }
        return true;
    }

    size_t udp_source_impl::dataAvailable() {
    	// Get amount of data available
    	boost::asio::socket_base::bytes_readable command(true);
    	udpsocket->io_control(command);
    	size_t bytes_readable = command.get();

    	return (bytes_readable+localQueue.size());
    }

    size_t udp_source_impl::netDataAvailable() {
    	// Get amount of data available
    	boost::asio::socket_base::bytes_readable command(true);
    	udpsocket->io_control(command);
    	size_t bytes_readable = command.get();

    	return bytes_readable;
    }


    int
    udp_source_impl::work_test(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
        gr::thread::scoped_lock guard(d_mutex);

    	int bytesAvailable = netDataAvailable();

    	// quick exit if nothing to do
        if ((bytesAvailable == 0) && (localQueue.size() == 0))
        	return 0;

        char *out = (char *) output_items[0];
    	int bytesRead;
    	int returnedItems;
    	int localNumItems;
        int i;
    	unsigned int numRequested = noutput_items * d_block_size;

    	// we could get here even if no data was received but there's still data in the queue.
    	// however read blocks so we want to make sure we have data before we call it.
    	if (bytesAvailable > 0) {
    		int bytesToGet;
    		if (bytesAvailable > numRequested)
    			bytesToGet=numRequested;
    		else
    			bytesToGet=bytesAvailable;

            boost::asio::streambuf::mutable_buffers_type buf = read_buffer.prepare(bytesToGet);
        	// http://stackoverflow.com/questions/28929699/boostasio-read-n-bytes-from-socket-to-streambuf
            bytesRead = udpsocket->receive_from(buf,d_endpoint);

            if (bytesRead > 0) {
                read_buffer.commit(bytesRead);

                // Get the data and add it to our local queue.  We have to maintain a local queue
                // in case we read more bytes than noutput_items is asking for.  In that case
                // we'll only return noutput_items bytes
                const char *readData = boost::asio::buffer_cast<const char*>( read_buffer.data());

                int blocksRead=bytesRead / d_block_size;
                int remainder = bytesRead % d_block_size;

                if ((localQueue.size()==0) && (remainder==0)) {
                	// If we don't have any data in the current queue,
                	// and in=out, we'll just move the data and exit.  It's faster.
                	unsigned int qnoi = blocksRead * d_block_size;
                	for (i=0;i<qnoi;i++) {
                		out[i]=readData[i];
                	}

                	read_buffer.consume(bytesRead);

                	return blocksRead;
                }
                else {
                    for (i=0;i<bytesRead;i++) {
                    	localQueue.push(readData[i]);
                    }
                	read_buffer.consume(bytesRead);
                }
            }
    	}

    	// let's figure out how much we have in relation to noutput_items
        localNumItems = localQueue.size() / d_block_size;

        // This takes care of if we have more data than is being requested
        if (localNumItems >= noutput_items) {
        	localNumItems = noutput_items;
        }

        // Now convert our block back to bytes
    	unsigned int noi = localNumItems * d_block_size;  // block size is sizeof(item) * vlen

    	for (i=0;i<noi;i++) {
    		out[i]=localQueue.front();
    		localQueue.pop();
    	}

    	// If we had less data than requested, it'll be reflected in the return value.
        return localNumItems;
    }

    int
    udp_source_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
        gr::thread::scoped_lock guard(d_mutex);

    	int bytesAvailable = netDataAvailable();

    	// quick exit if nothing to do
        if ((bytesAvailable == 0) && (localQueue.size() == 0))
        	return 0;

        char *out = (char *) output_items[0];
    	int bytesRead;
    	int returnedItems;
    	int localNumItems;
        int i;
    	unsigned int numRequested = noutput_items * d_block_size;

    	// we could get here even if no data was received but there's still data in the queue.
    	// however read blocks so we want to make sure we have data before we call it.
    	if (bytesAvailable > 0) {
    		int bytesToGet;
    		if (bytesAvailable > numRequested)
    			bytesToGet=numRequested;
    		else
    			bytesToGet=bytesAvailable;

            boost::asio::streambuf::mutable_buffers_type buf = read_buffer.prepare(bytesToGet);
        	// http://stackoverflow.com/questions/28929699/boostasio-read-n-bytes-from-socket-to-streambuf
            bytesRead = udpsocket->receive_from(buf,d_endpoint);

            if (bytesRead > 0) {
                read_buffer.commit(bytesRead);

                // Get the data and add it to our local queue.  We have to maintain a local queue
                // in case we read more bytes than noutput_items is asking for.  In that case
                // we'll only return noutput_items bytes
                const char *readData = boost::asio::buffer_cast<const char*>( read_buffer.data());

                int blocksRead=bytesRead / d_block_size;
                int remainder = bytesRead % d_block_size;

                if ((localQueue.size()==0) && (remainder==0)) {
                	// If we don't have any data in the current queue,
                	// and in=out, we'll just move the data and exit.  It's faster.
                	unsigned int qnoi = blocksRead * d_block_size;
                	for (i=0;i<qnoi;i++) {
                		out[i]=readData[i];
                	}

                	read_buffer.consume(bytesRead);

                	return blocksRead;
                }
                else {
                    for (i=0;i<bytesRead;i++) {
                    	localQueue.push(readData[i]);
                    }
                	read_buffer.consume(bytesRead);
                }
            }
    	}

    	// let's figure out how much we have in relation to noutput_items
        localNumItems = localQueue.size() / d_block_size;

        // This takes care of if we have more data than is being requested
        if (localNumItems >= noutput_items) {
        	localNumItems = noutput_items;
        }

        // Now convert our block back to bytes
    	unsigned int noi = localNumItems * d_block_size;  // block size is sizeof(item) * vlen

    	for (i=0;i<noi;i++) {
    		out[i]=localQueue.front();
    		localQueue.pop();
    	}

    	// If we had less data than requested, it'll be reflected in the return value.
        return localNumItems;
    }
  } /* namespace grsbt */
} /* namespace gr */

