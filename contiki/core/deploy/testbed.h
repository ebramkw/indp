/*
 * Copyright (c) 2011, ETH Zurich.
 * Copyright (c) 2013, Olaf Landsiedel.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Olaf Landsiedel <olafl@chalmers.se>
 * Author: Federico Ferrari <ferrari@tik.ee.ethz.ch>
 *
 */

/**
 * \file
 *         Testbed configurations.
 * \author
 *         Olaf Landsiedel <olafl@chalmers.se>
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#ifndef TESTBED_H_
#define TESTBED_H_

#include <stdint.h>

#define indriya 1
#define motelab 2
#define twist 3
#define flocklab 4
#define kansei 5

// #define TESTBED flocklab
// #define TESTBED indriya
#define COOJA 1

extern uint16_t node_index;

#ifdef TESTBED

#if TESTBED == indriya
//#warning "compiling for indriya"
#define COOJA 0
// #define CHAOS_NODES 4
// #define NODE_ID_MAPPING {1, 2, 3, 4}
// #define CHAOS_NODES 10
// #define NODE_ID_MAPPING {1, 2, 3, 4, 5, 6, 8, 9, 10, 11} // Clustered
// #define NODE_ID_MAPPING {1, 16, 28, 36, 44, 57, 68, 84, 120, 132} // Distributed
// #define CHAOS_NODES 15
// #define NODE_ID_MAPPING {1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}
// #define CHAOS_NODES 20
// #define NODE_ID_MAPPING {1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21}
// #define CHAOS_NODES 18
// #define NODE_ID_MAPPING {1, 38, 39, 41, 42, 43, 45, 46, 47, 50, 56, 63, 66, 67, 68, 69, 70, 71}
// #define CHAOS_NODES 30
// #define NODE_ID_MAPPING {1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 24, 25, 26, 27, 28, 30, 31, 32, 34, 35, 36, 4386}
// #define NODE_ID_MAPPING {1, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 50, 52, 57, 60, 64, 68, 72, 76, 80, 84, 117, 120, 124, 128, 132, 136, 139}
// #define CHAOS_NODES 40
// #define NODE_ID_MAPPING {1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 21, 24, 25, 26, 27, 28, 30, 31, 32, 33, 34, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46}

// ebramkw

// 32
// #define MERGE 3
// #define TOTAL_NODES 32
// #define NODE_ID_MAPPING {4, 6, 7, 8, 9, 13, 16, 17, 18, 19, 20, 22, 24, 25, 26, 27, 29, 30, 31, 34, 35, 37, 39, 40, 42, 43, 44, 45, 46, 47, 48, 49}
// #define INITIATOR_NODE_ID 17

// 8
// #define MERGE 3
// #define TOTAL_NODES 32
// #define NODE_ID_MAPPING {4, 7, 8, 9, 12, 13, 16, 17}
// #define INITIATOR_NODE_ID 17

// ********************************************************************************************* //
// nodes with sensors!
// 32, 14, 20, 34, 10, 1, 12, 60, 61, 2, 15, 19, 37, 26, 13, 50, 24
// ********************************************************************************************* //

// 16
//***change JUST_STARTED to 6***//
// exclude from burn: 27
// bad connectivity: 6, 7, 13
// Watchdog reset: 16
// #define MERGE 1
// #define CHAOS_NODES 16
// #define NODE_ID_MAPPING {16, 17, 18, 19, 22, 23, 24, 26, 40, 41, 42, 44, 46, 47, 48, 52}
// #define INITIATOR_NODE_ID 16

#define MERGE 1
#define CHAOS_NODES 30
#define NODE_ID_MAPPING {16, 17, 18, 19, 22, 23, 24, 27, 28, 29, 30, 35, 40, 42, 44, 46, 47, 48, 49, 51, 53, 54, 55, 63, 66, 69, 70, 71, 72, 74}
#define INITIATOR_NODE_ID 16


// 40
// #define TOTAL_NODES 40
// #define NODE_ID_MAPPING {4, 6, 7, 8, 9, 13, 16, 17, 18, 19, 20, 22, 24, 25, 26, 27, 29, 30, 31, 32, 33, 34, 35, 37, 39, 40, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55}
// #define INITIATOR_NODE_ID 17

// #define MERGE 3
// #define TOTAL_NODES 48
// #define NODE_ID_MAPPING {4, 6, 7, 8, 12, 13, 16, 17}
// #define INITIATOR_NODE_ID 17

// 48
//***change JUST_STARTED to 6***//
// #define MERGE 3
//#define TOTAL_NODES 48
//#define NODE_ID_MAPPING {4, 6, 7, 8, 12, 13, 16, 17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 31, 34, 35, 37, 39, 40, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 54, 55, 57, 60, 61, 62, 63, 65, 66, 67, 68, 69}
//#define INITIATOR_NODE_ID 17

// power measurment
// 48
// #define MERGE 3
// #define TOTAL_NODES 48
// // first node is the guy connected to the Monsoon power meter
// #define NODE_ID_MAPPING {621, 6, 7, 8, 12, 13, 16, 17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 31, 34, 35, 37, 39, 40, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 54, 55, 57, 58, 60, 61, 62, 63, 65, 66, 67, 68}
// #define INITIATOR_NODE_ID 17

// 56
// #define TOTAL_NODES 56
// #define NODE_ID_MAPPING {4, 6, 9, 13, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40, 42, 43, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 55, 56, 57, 58, 60, 61, 62, 63, 64, 65, 67, 68, 69, 70, 71, 72, 73}
// #define INITIATOR_NODE_ID 17

// 64
// #define TOTAL_NODES 64
// #define NODE_ID_MAPPING {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69}


// #define NODE_ID_MAPPING {17, 18, 19, 21, 22, 24, 25, 26}
// #define INITIATOR_NODE_ID 17

// #define CHAOS_NODES 24
// #define NODE_ID_MAPPING {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25}

// #define CHAOS_NODES 8
// #define NODE_ID_MAPPING {3, 10, 16, 38, 39, 41, 43, 47, 50, 56, 63, 66, 67, 69, 70, 71}

// #define CHAOS_NODES 26
// #define NODE_ID_MAPPING {3, 10, 16, 37, 38, 40, 41, 43, 44, 50, 53, 56, 63, 65, 66, 70, 71, 73, 75, 115, 121, 122, 123, 124, 131, 621}

// #define NODE_ID_MAPPING {50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65}
// #define INITIATOR_NODE_ID 50

// ebramkw
// motes with sensors
// #define NODE_ID_MAPPING {1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21}

// #define TINYOS_SERIAL_FRAMES 1
// #ifndef INITIATOR_NODE_ID
// #define INITIATOR_NODE_ID 41
// #endif

//#if TESTBED == indriya
////#warning "compiling for indriya"
//#define CHAOS_NODES 100
//#define COOJA 0
//#define NODE_ID_MAPPING {1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 24, 25, 26, 27, 28, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 55, 56, 58, 59, 60, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 127, 128, 129, 130, 131, 132, 133, 135, 136, 137, 138, 139}
//#define TINYOS_SERIAL_FRAMES 1
//#ifndef INITIATOR_NODE_ID
//#define INITIATOR_NODE_ID 1
//#endif

//#if TESTBED == indriya
////#warning "compiling for indriya"
//#define CHAOS_NODES 134
//#define COOJA 0
//#define NODE_ID_MAPPING {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 96, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139}
//#define TINYOS_SERIAL_FRAMES 1
//#ifndef INITIATOR_NODE_ID
//#define INITIATOR_NODE_ID 1
//#endif

#elif TESTBED == motelab
//#warning "compiling for motelab"
#define CHAOS_NODES 61
#define COOJA 0
#define TINYOS_SERIAL_FRAMES 1
#define NODE_ID_MAPPING {1, 2, 3, 4, 17, 18, 19, 20, 21, 22, 27, 28, 31, 32, 41, 42, 45, 46, 53, 57, 58, 63, 64, 65, 66, 67, 68, 75, 76, 79, 80, 81, 87, 88, 103, 104, 107, 108, 109, 110, 113, 114, 115, 116, 117, 118, 125, 129, 130, 161, 162, 167, 168, 169, 170, 173, 174, 181, 182, 183, 184}
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID 103
#endif


#elif TESTBED == twist
//#warning "compiling for twist"
#define CHAOS_NODES 85
#define COOJA 0
#define TINYOS_SERIAL_FRAMES 1
#define NODE_ID_MAPPING {10, 11, 12, 13, 15, 79, 80, 81, 82, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 99, 100, 101, 102, 103, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 185, 186, 188, 191, 192, 193, 194, 195, 196, 197, 199, 200, 202, 203, 205, 206, 207, 209, 211, 212, 213, 214, 216, 218, 220, 221, 223, 224, 225, 228, 230, 231, 240, 241, 249, 250, 251, 252, 262, 272}
// #define NODE_ID_MAPPING {10, 11, 12, 13, 15, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 99, 100, 101, 102, 103, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 202, 203, 204, 205, 206, 207, 208, 209, 211, 212, 213, 214, 215, 216, 218, 220, 221, 222, 223, 224, 225, 228, 229, 230, 231, 240, 241, 249, 250, 251, 252, 262, 272}
//#define NODE_ID_MAPPING {10, 11, 12, 13, 15, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 95, 96, 97, 99, 100, 101, 102, 103, 137, 138, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 202, 203, 206, 207, 208, 209, 211, 212, 213, 214, 215, 216, 218, 220, 221, 222, 223, 224, 225, 228, 229, 230, 231, 240, 241, 249, 250, 251, 252, 262, 272}
// #define NODE_ID_MAPPING {10, 11, 12, 13, 15, 79, 80, 81, 82, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 99, 100, 101, 102, 103, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 185, 186, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 199, 200, 202, 203, 205, 206, 207, 208, 209, 211, 212, 213, 214, 216, 218, 220, 221, 222, 223, 224, 225, 228, 230, 231, 240, 241, 249, 250, 251, 252, 262, 272}
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID 192
#endif

#elif TESTBED == fit
//#warning "compiling for twist"
#define CHAOS_NODES 30
#define COOJA 0
#define NODE_ID_MAPPING {2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32}
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID 2
#endif

#elif TESTBED == flocklab
#define MERGE 1
//#warning "compiling for flocklab" 
//select tinyOS
#define COOJA 0
#define TINYOS_SERIAL_FRAMES 0
// #define CHAOS_NODES 30
// #define NODE_ID_MAPPING {1, 2, 3, 4, 6, 7, 8, 10, 11, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33, 200, 201, 202, 204}
// #define NODE_ID_MAPPING {1, 2, 3, 4, 6, 7, 8, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33, 200, 202, 204}
// #define CHAOS_NODES 20
// #define NODE_ID_MAPPING {1, 3, 4, 7, 8, 11, 13, 15, 16, 18, 19, 22, 23, 25, 26, 28, 31, 33, 200, 204}

// ebramkw
// total number of node = 27

//***comment TESTBED 1***//

#define CHAOS_NODES 16
// #define TOTAL_NODES 24
// don't use: 2, 4, 27
#define NODE_ID_MAPPING {1, 3, 6, 8, 10, 15, 16, 18, 22, 23, 24, 26, 28, 31, 32, 33}

// #define NODE_ID_MAPPING {1, 3, 4, 6, 8, 10, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33}
// #define NODE_ID_MAPPING {1, 3, 4, 8, 15, 31, 32, 33}

// #define CHAOS_NODES 16
// #define NODE_ID_MAPPING {1, 3, 4, 6, 8, 15, 16, 18, 22, 23, 24, 27, 28, 31, 32, 33}

// ebramkw
// 001 002 003 004 006 007 008 010 011 013 014 015 016 017 018 019 020 022 023 024 025 026 027 028 031 032 033 201

// #define CHAOS_NODES 10
// #define NODE_ID_MAPPING {1, 4, 8, 13, 16, 19, 23, 26, 31, 200}
//#define NODE_ID_MAPPING {1, 2, 4, 6, 8, 10, 15, 16, 22, 28, 33, 200, 202, 204}
//#define NODE_ID_MAPPING {1, 2, 4, 8, 15}
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID 1
#endif
#ifndef CC2420_TXPOWER
#define CC2420_TXPOWER 31
#endif

#elif TESTBED == kansei
//#warning "compiling for kansei"
#define CHAOS_NODES 368
#define COOJA 0
#define TINYOS_SERIAL_FRAMES 1
//101-460, 469-476
#define NODE_ID_MAPPING {101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,263,264,265,266,267,268,269,270,271,272,273,274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,323,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,469,470,471,472,473,474,475,476}
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID 101
#endif

#else
#error "unknown testbed config"
#endif
#endif

#endif /* TESTBED_H_ */

