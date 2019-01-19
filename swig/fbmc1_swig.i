/* -*- c++ -*- */

#define FBMC1_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "fbmc1_swig_doc.i"

%{
#include "fbmc1/fbmc_mac_encoder.h"
#include "fbmc1/fbmc_phy_encoder.h"
#include "fbmc1/fbmc_carrier_allocator.h"
#include "fbmc1/fbmc_oqam.h"
#include "fbmc1/fbmc_p2s.h"
#include "fbmc1/fbmc_equalizer.h"
#include "fbmc1/fbmc_oqam_recv.h"
#include "fbmc1/fbmc_phy_decoder.h"
#include "fbmc1/fbmc_add.h"
#include "fbmc1/fbmc_ppn.h"
#include "fbmc1/fbmc_ppn_recv.h"
#include "fbmc1/fbmc_valve_fd.h"
#include "fbmc1/fbmc_pdu_to_tagged_stream.h"
#include "fbmc1/fbmc_norm_akf_fd.h"
#include "fbmc1/fbmc_sample_collector.h"
#include "fbmc1/fbmc_norm_akf_td.h"
#include "fbmc1/fbmc_mac_decoder.h"
#include "fbmc1/fbmc_payload_generator.h"
#include "fbmc1/fbmc_tagged_stream_to_pdu.h"
#include "fbmc1/fbmc_sensor.h"
#include "fbmc1/fbmc_valve_td.h"
#include "fbmc1/fbmc_equ_oqam_phy.h"
#include "fbmc1/fbmc_log_chann_state.h"
%}

%include "fbmc1/fbmc_mac_encoder.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_mac_encoder);
%include "fbmc1/fbmc_phy_encoder.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_phy_encoder);
%include "fbmc1/fbmc_carrier_allocator.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_carrier_allocator);
%include "fbmc1/fbmc_oqam.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_oqam);

%include "fbmc1/fbmc_p2s.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_p2s);

%include "fbmc1/fbmc_equalizer.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_equalizer);
%include "fbmc1/fbmc_oqam_recv.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_oqam_recv);
%include "fbmc1/fbmc_phy_decoder.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_phy_decoder);

%include "fbmc1/fbmc_add.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_add);


%include "fbmc1/fbmc_ppn.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_ppn);
%include "fbmc1/fbmc_ppn_recv.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_ppn_recv);


%include "fbmc1/fbmc_valve_fd.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_valve_fd);

%include "fbmc1/fbmc_pdu_to_tagged_stream.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_pdu_to_tagged_stream);
%include "fbmc1/fbmc_norm_akf_fd.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_norm_akf_fd);

%include "fbmc1/fbmc_sample_collector.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_sample_collector);


%include "fbmc1/fbmc_norm_akf_td.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_norm_akf_td);

%include "fbmc1/fbmc_mac_decoder.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_mac_decoder);

%include "fbmc1/fbmc_payload_generator.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_payload_generator);
%include "fbmc1/fbmc_tagged_stream_to_pdu.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_tagged_stream_to_pdu);
%include "fbmc1/fbmc_sensor.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_sensor);
%include "fbmc1/fbmc_valve_td.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_valve_td);
%include "fbmc1/fbmc_equ_oqam_phy.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_equ_oqam_phy);

%include "fbmc1/fbmc_log_chann_state.h"
GR_SWIG_BLOCK_MAGIC2(fbmc1, fbmc_log_chann_state);
