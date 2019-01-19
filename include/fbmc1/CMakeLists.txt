# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Install public header files
########################################################################
install(FILES
    api.h
    fbmc_mac_encoder.h
    fbmc_phy_encoder.h
    fbmc_carrier_allocator.h
    fbmc_oqam.h
    fbmc_p2s.h
    fbmc_equalizer.h
    fbmc_oqam_recv.h
    fbmc_phy_decoder.h
    fbmc_add.h
    fbmc_ppn.h
    fbmc_ppn_recv.h
    fbmc_valve_fd.h
    fbmc_pdu_to_tagged_stream.h
    fbmc_norm_akf_fd.h
    fbmc_sample_collector.h
    fbmc_norm_akf_td.h
    fbmc_mac_decoder.h
    fbmc_payload_generator.h
    fbmc_tagged_stream_to_pdu.h
    fbmc_sensor.h
    fbmc_valve_td.h
    fbmc_equ_oqam_phy.h
    fbmc_log_chann_state.h DESTINATION include/fbmc1
)
