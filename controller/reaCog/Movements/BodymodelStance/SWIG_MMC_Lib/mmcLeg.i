/* File : mmcLeg.i */
%module mmcLeg

%{
#include "mmcLeg.h"
%}

%include "typemaps.i"
%include "std_vector.i"
// Instantiate templates used by example
namespace std {
   %template(DoubleVector) vector<double>;
}

/* Let's just grab the original header file here */
%include "mmcLeg.h"

