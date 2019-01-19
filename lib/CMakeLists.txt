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
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND fbmc1_sources
    #-------------------
    #-------------------
    #-------------------
    utils/debug.cc
    utils/constellations.cc     
    utils/logical_channel.cc
    utils/preamble_set_freq_domain.cc
    utils/preamble_set_time_domain.cc
    utils/receiver_tag.cc
    utils/filter_function_fbmc.cc
    sensor/scheduler.cc
    sensor/analyser.cc
    sensor/sensor.cc
    sensor/extractor.cc
    sensor/subroutines.cc
    mac_coding/encoder.cc
    mac_coding/decoder.cc
    phy_coding/encoder.cc
    phy_coding/decoder.cc
    phy_coding/ieee/ieee.cc
    phy_coding/ieee/spiral_full_spiral.cc  
    phy_coding/ieee/verify_viterbi.cc    
    #-------------------
    #-------------------
    #-------------------
    fbmc_mac_encoder_impl.cc
    fbmc_phy_encoder_impl.cc
    fbmc_carrier_allocator_impl.cc
    fbmc_oqam_impl.cc
    fbmc_p2s_impl.cc
    fbmc_equalizer_impl.cc
    fbmc_oqam_recv_impl.cc
    fbmc_phy_decoder_impl.cc
    fbmc_add_impl.cc
    fbmc_ppn_impl.cc
    fbmc_ppn_recv_impl.cc
    fbmc_valve_fd_impl.cc
    fbmc_pdu_to_tagged_stream_impl.cc
    fbmc_norm_akf_fd_impl.cc
    fbmc_sample_collector_impl.cc
    fbmc_norm_akf_td_impl.cc
    fbmc_mac_decoder_impl.cc
    fbmc_payload_generator_impl.cc
    fbmc_tagged_stream_to_pdu_impl.cc
    fbmc_sensor_impl.cc
    fbmc_valve_td_impl.cc
    fbmc_equ_oqam_phy_impl.cc
    fbmc_log_chann_state_impl.cc )

set(fbmc1_sources "${fbmc1_sources}" PARENT_SCOPE)
if(NOT fbmc1_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT fbmc1_sources)

add_library(gnuradio-fbmc1 SHARED ${fbmc1_sources})
target_link_libraries(gnuradio-fbmc1 ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES})
set_target_properties(gnuradio-fbmc1 PROPERTIES DEFINE_SYMBOL "gnuradio_fbmc1_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-fbmc1 PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-fbmc1
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_fbmc1_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_fbmc1.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_fbmc1.cc
)

add_executable(test-fbmc1 ${test_fbmc1_sources})

target_link_libraries(
  test-fbmc1
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  gnuradio-fbmc1
)

GR_ADD_TEST(test_fbmc1 test-fbmc1)
