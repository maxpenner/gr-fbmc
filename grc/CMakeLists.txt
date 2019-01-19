# Copyright 2011 Free Software Foundation, Inc.
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
install(FILES
    fbmc1_fbmc_mac_encoder.xml
    fbmc1_fbmc_phy_encoder.xml
    fbmc1_fbmc_carrier_allocator.xml
    fbmc1_fbmc_oqam.xml
    fbmc1_fbmc_p2s.xml
    fbmc1_fbmc_equalizer.xml
    fbmc1_fbmc_oqam_recv.xml
    fbmc1_fbmc_phy_decoder.xml
    fbmc1_fbmc_add.xml
    fbmc1_fbmc_ppn.xml
    fbmc1_fbmc_ppn_recv.xml
    fbmc1_fbmc_valve_fd.xml
    fbmc1_fbmc_pdu_to_tagged_stream.xml
    fbmc1_fbmc_norm_akf_fd.xml
    fbmc1_fbmc_sample_collector.xml
    fbmc1_fbmc_norm_akf_td.xml
    fbmc1_fbmc_mac_decoder.xml
    fbmc1_fbmc_payload_generator.xml
    fbmc1_fbmc_tagged_stream_to_pdu.xml
    fbmc1_fbmc_sensor.xml
    fbmc1_fbmc_valve_td.xml
    fbmc1_fbmc_equ_oqam_phy.xml
    fbmc1_fbmc_log_chann_state.xml DESTINATION share/gnuradio/grc/blocks
)
