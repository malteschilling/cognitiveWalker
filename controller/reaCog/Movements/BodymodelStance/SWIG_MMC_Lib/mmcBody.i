/* File : mmcBody.i */
%module mmcBody

%{
#include "mmcBody.h"
%}

%include "typemaps.i"
%include "std_vector.i"
// Instantiate templates used by example
namespace std {
   %template(DoubleVector) vector<double>;
   %template(DoubleVectorVector) vector< vector<double> >;
}

/* Let's just grab the original header file here */
%include "mmcBody.h"

