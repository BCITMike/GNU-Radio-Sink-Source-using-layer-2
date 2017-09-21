/* -*- c++ -*- */

#define GRSBT_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "grsbt_swig_doc.i"

%{
#include "grsbt/udp_sink.h"
#include "grsbt/udp_source.h"
%}


%include "grsbt/udp_sink.h"
GR_SWIG_BLOCK_MAGIC2(grsbt, udp_sink);

%include "grsbt/udp_source.h"
GR_SWIG_BLOCK_MAGIC2(grsbt, udp_source);
