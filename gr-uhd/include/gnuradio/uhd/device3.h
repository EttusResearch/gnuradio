/* -*- c++ -*- */
/* 
 * Copyright 2014 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_UHD_DEVICE3_H
#define INCLUDED_UHD_DEVICE3_H

#include <gnuradio/uhd/api.h>
#include <uhd/types/device_addr.hpp>

#ifndef INCLUDED_UHD_USRP_MULTI_USRP_HPP
namespace uhd {
  namespace usrp {
    class multi_usrp;
  }
}
#endif

namespace gr {
  namespace uhd {

/*!
 * \brief A representation of a generation 3 USRP device.
 */
class UHD_API device3
{
 public:
  typedef boost::shared_ptr<device3> sptr;
  virtual ~device3() {};

  //! Return a pointer to the underlying multi_usrp device
  virtual boost::shared_ptr< ::uhd::usrp::multi_usrp > get_device(void) = 0;

  virtual void connect(const std::string &block1, size_t src_block_port, const std::string block2, size_t dst_block_port) = 0;

  static sptr make(
      const ::uhd::device_addr_t &device_addr
  );

};

  } // namespace uhd
} // namespace gr

#endif /* INCLUDED_UHD_DEVICE3_H */

