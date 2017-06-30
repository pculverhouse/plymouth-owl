#ifndef OWLPWM_HPP
#define OWLPWM_HPP

#endif // OWLPWM_HPP

//* copyright Phil Culverhouse, Centre for Robotics & Nenural Systems, University of Plymouth, 2016
*
*
*    Permission is granted to copy, distribute and/or modify this document
*    under the terms of the GNU Free Documentation License, Version 1.3
*    or any later version published by the Free Software Foundation;
*    with no Invariant Sections.
*
// Defines for OWL servo limits
// PFC Owl robot
//
using namespace std;

// OWL eye ranges (max)
int RyBm = 1135; // (bottom) to
int RyTm = 2000; //(top)
int RxRm = 1890; //(right) to
int RxLm = 1200; //(left)
int LyBm = 2000; //(bottom) to
int LyTm = 1195; //(top)
int LxRm = 1850; // (right) to
int LxLm = 1200; // (left)
int NeckR = 1105;
int NeckL = 1940;
// VGA match ranges
int RyBv = 1240; // (bottom) to
int RyTv = 1655; //(top)
int RxRv = 1845; //(right) to
int RxLv = 1245; //(left)
int LyBv = 1880; //(bottom) to
int LyTv = 1420; //(top)
int LxRv = 1835; // (right) to
int LxLv = 1265; // (left)
int RxC=1470;
int RyC=1445;
int LxC=1485;
int LyC=1535;
int NeckC = 1520;
int Ry,Rx,Ly,Lx,Neck; // calculate values for position
//MAX servo eye socket ranges
int RyRangeM=RyTm-RyBm;
int RxRangeM=RxRm-RxLm;
int LyRangeM=LyTm-LyBm; // reflected so negative
int LxRangeM=LxRm-LxLm;
int NeckRange=NeckL-NeckR;
//vga CAMERA ranges
int RyRangeV=RyTv-RyBv;
int RxRangeV=RxRv-RxLv;
int LyRangeV=LyTv-LyBv; // reflected so negative
int LxRangeV=LxRv-LxLv;
