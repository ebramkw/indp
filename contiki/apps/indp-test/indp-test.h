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
 * Author: Federico Ferrari <ferrari@tik.ee.ethz.ch>
 * Author: Olaf Landsiedel <olafl@chalmers.se>
 *
 */

/**
 * \defgroup chaos-test Simple application for testing Chaos
 * @{
 */

/**
 * \file
 *         A simple example of an application that uses Chaos, header file.
 *
 *         The application schedules Chaos periodically.
 *         The period is determined by \link CHAOS_PERIOD \endlink.
 * \author
 *         Olaf Landsiedel <olafl@chalmers.se>
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#ifndef CHAOS_TEST_H_
#define CHAOS_TEST_H_

#include "chaos.h"
#include "node-id.h"
#include "cc2420.h"

/**
 * \defgroup chaos-test-settings Application settings
 * @{
 */

/**
 * \brief NodeId of the initiator.
 *        Default value: 200
 */
#ifndef INITIATOR_NODE_ID
//#define INITIATOR_NODE_ID       200
#define INITIATOR_NODE_ID       1
#endif

/**
 * \brief Maximum number of transmissions N.
 *        Default value: 255.
 */
//#define N_TX                    5
#define N_TX                    255

/**
 * \brief Maximum number of transmissions when complete.
 *        Default value: 5.
 */
#ifndef N_TX_COMPLETE
#define N_TX_COMPLETE 						5
#endif

/**
 * \brief define number of nodes (if not testbed config is used)
 *        Default value: 3.
 */

// #ifndef TESTBED
#ifndef CHAOS_NODES
#define CHAOS_NODES 30
//#define CHAOS_NODES 5
//#define CHAOS_NODES 8
//#define CHAOS_NODES 32
#endif
// #endif

#define NODES_SEND_DATA 1
// extern uint8_t data_sender;

#define TOTAL_NODES_RUN 0

#if TOTAL_NODES_RUN
	// check on setdata and ofge_init()
	#ifndef TOTAL_NODES
	#define TOTAL_NODES 48
	#endif
	#define TOTAL_MERGE_LEN ((TOTAL_NODES / 8) + ((TOTAL_NODES % 8) ? 1 : 0))
#endif

/**
 * \brief compute length of the flags array.
 */
#define MERGE_LEN ((CHAOS_NODES / 8) + ((CHAOS_NODES % 8) ? 1 : 0))

/**
 * \brief compute how the last flag byte looks when we are complete (all other flag bytes are 0xFF)
 */
#define CHAOS_COMPLETE_FLAG ((1 << (((CHAOS_NODES - 1) % 8) + 1)) - 1)

/**
 * \brief Period with which a Chaos phase is scheduled.
 *        Default value: 2000 ms (if IPI is not defined)
 */
//#define CHAOS_PERIOD           (RTIMER_SECOND / 4)      // 250 ms
//#define CHAOS_PERIOD           (RTIMER_SECOND / 2)      // 500 ms
//#define CHAOS_PERIOD           (RTIMER_SECOND / 1)      // 1000 ms
#ifdef IPI_IN_SEC
#define CHAOS_PERIOD           ((uint32_t)RTIMER_SECOND*IPI_IN_SEC)
#else
#define CHAOS_PERIOD           ((uint32_t)RTIMER_SECOND * 2)     //  2000 ms
#endif


/**
 * \brief Duration of each Chaos phase.
 *        Default value: 1500 ms (if duration divider is not defined).
 */
//#define CHAOS_DURATION         (RTIMER_SECOND / 50)     //  20 ms
//#define CHAOS_DURATION         (RTIMER_SECOND / 10)     //  100 ms
#ifdef DURATION_DIVIDER
#define CHAOS_DURATION         (RTIMER_SECOND / DURATION_DIVIDER)
#else
#define CHAOS_DURATION         ((uint32_t)RTIMER_SECOND * 3 / 2)     //  1.5 ms
#endif

/**
 * \brief Guard-time at receivers.
 *        Default value: 1000 us (3000 us when using Cooja).
 */
#if COOJA
#define CHAOS_GUARD_TIME       (RTIMER_SECOND / 333)	// 3ms = 3000 us
#else
#define CHAOS_GUARD_TIME       (RTIMER_SECOND / 1000)   // 1ms = 1000 us
#endif /* COOJA */

/**
 * \brief Guard-time during bootstrapping at receivers.
 *        Default value: 50 ms.
 */
#define CHAOS_INIT_GUARD_TIME  (RTIMER_SECOND / 20)		// 50 ms 

/**
 * \brief Number of consecutive Chaos phases with successful computation of reference time required to exit from bootstrapping.
 *        Default value: 3.
 */
#define CHAOS_BOOTSTRAP_PERIODS 3

/**
 * \brief Duration during bootstrapping at receivers. Duration+49ms?
 *        Default value: 59.474 ms.
 */
#define CHAOS_INIT_DURATION    (CHAOS_DURATION - CHAOS_GUARD_TIME + CHAOS_INIT_GUARD_TIME) 	//  59.474 ms M: ?? ==> 1549ms

/**
 * \brief Period during bootstrapping at receivers. Duration+59ms
 *        It should not be an exact fraction of \link CHAOS_PERIOD \endlink.
 *        Default value: 69.474 ms.
 */
#define CHAOS_INIT_PERIOD      (CHAOS_INIT_DURATION + RTIMER_SECOND / 100)                   //  69.474 ms M: ?? ==> 1559ms

/**
 * \defgroup ebramkw Variables
 * @{
 */

#define M 6				//number of data matrix rows (samples)
#define N 3					//number of data matrix columns (features)

#define SHARE_SV 1			//number of singular vectors and singular values to share between nodes

// float singular_values[N];		//eigenvalues vector
// float singular_vectors[N][N];	//eigenvectors matrix
float singular_value;			//eigenvalue
float singular_vector[N];		//eigenvector
// float global_singular_vectors[N][N]; //global
float global_singular_vector[N]; 	//global

// int data_round;
// int phase;
// float total_projected_data_matrix[CHAOS_NODES][M][SHARE_SV];
// float total_approximated_data_matrix[CHAOS_NODES][M][N];
float data_matrix[M][N];

float local_pca[N + 1];

// To compare current and previous round local PCA
#define R_PCA 1

// To stop once get data solved and don't care about the no_update
// To know how much data, will use a data bit vector
#if R_PCA
	#define LOSSY_RPCA 1
	#if LOSSY_RPCA
		int actual_num_no_update;
		int actual_num_solved;
	#endif
#endif

/** @} */

/**
 * \brief payload length.
 *        Default value: 100 bytes.
 */

extern uint8_t periodic_mode;
#define MAX_PERIODIC_ROUNDS 2
extern uint8_t curr_periodic_rounds;

// ebramkw: channel hopping
#define CHANNEL_HOPPING 0
#if CHANNEL_HOPPING
	#define NUM_OF_100_DURATION 8
#endif


// chaos duty cyclying
#define RADIO_ON_DURATION ((uint32_t)RTIMER_SECOND / 5) // 200 ms
// #define RADIO_OFF_DURATION ((uint32_t)RTIMER_SECOND / 10) // 100 ms
extern uint8_t channel_is_free;

#define WAKE_UP_NETWORK_NUM 10
#define SRC_RETRANSMISSION_ON_DURATION ((uint32_t)RTIMER_SECOND / 50) // 10 ms
#define SRC_RETRANSMISSION_OFF_DURATION ((uint32_t)RTIMER_SECOND / 50) // 20 ms
extern uint8_t src_got_answer;

#define FIXED_RADIO_OFF_DURATION ((uint32_t)RTIMER_SECOND * 2) // 2s


// sampling duty cycling
#define NUM_OF_SAMPLING 2
#define SAMPLING_RADIO_ON_DURATION ((uint32_t)RTIMER_SECOND / 1000) // 0.5 ms
#define SAMPLING_RADIO_OFF_DURATION ((uint32_t)RTIMER_SECOND * 3 / 2000) // 1.5 ms
#define SAMPLING_RADIO_LONG_OFF_DURATION ((uint32_t)RTIMER_SECOND / 20) // 50 ms
extern int cca;

#define ONE_SRC 0

// ebramkw: setup rounds
#define CHAOS_SETUP 1
#if CHAOS_SETUP
	#define SETUP_ROUNDS 10
	extern uint8_t curr_setup_rounds;
#endif

extern unsigned long control_total_secs;

#if ONE_SRC
extern unsigned long one_total_secs;
#endif


extern int curr_payload_len;

// change payload to 20 ==> size of one pca (N + 1) = 5 floats
#ifndef PAYLOAD_LEN
#define PAYLOAD_LEN 20	// ebramkw: define payload as multiple of 4 (number of float numbers)
// #define DATA_ROUNDS 1 // (M * N) / PAYLOAD_LEN ebramkw: how many rounds to share projected data
#endif

/**
 * \brief Data structure used to represent Chaos data.
 */
typedef struct {
	unsigned long seq_no; 			/**< Sequence number, incremented by the initiator at each Chaos phase. */
	#if R_PCA
		#if TOTAL_NODES_RUN
			uint8_t no_update[TOTAL_MERGE_LEN]; 	/**< showing if a node has no update in current local PCA, comparing to the previous round local PCA. */
		#else
			uint8_t no_update[MERGE_LEN];
		#endif
		// compute it from no_update flag to save size
		// int updated;					/**< updated so far */
		#if LOSSY_RPCA
			#if TOTAL_NODES_RUN
				uint8_t data_control[TOTAL_MERGE_LEN];
			#else
				uint8_t data_control[MERGE_LEN];
			#endif
		#endif
	#endif
	int source;						/**< Source node id */	
	int destination;				/**< Destination node id */	
	// solved can be removed and computed from solved_bv
	// int solved;						/**< Solved so far */
	// uint8_t flags[MERGE_LEN]; 		/**< Flags, showing which nodes already contributed. */
	uint8_t bv[MERGE_LEN]; 			/**< Bit Vectors, showing which packets composed the payload. */
	uint8_t solved_bv[MERGE_LEN]; 	/**< Bit Vectors, showing which packets are solved. */
	char payload[PAYLOAD_LEN]; 	/**< Payload, this is the application data. */
} chaos_data_struct;

/** @} */

/**
 * \defgroup chaos-test-defines Application internal defines
 * @{
 */

/**
 * \brief Length of data structure.
 */
#define DATA_LEN                    sizeof(chaos_data_struct)

/**
 * \brief synchronization on/off
 */
#define CHAOS_SYNC_MODE CHAOS_SYNC
//#define CHAOS_SYNC_MODE CHAOS_NO_SYNC

/**
 * \brief Check if the nodeId matches the one of the initiator.
 */
#define IS_INITIATOR()              (node_id == INITIATOR_NODE_ID)

/**
 * \brief Check if Chaos is still bootstrapping.
 * \sa \link CHAOS_BOOTSTRAP_PERIODS \endlink.
 */
#define CHAOS_IS_BOOTSTRAPPING()   (skew_estimated < CHAOS_BOOTSTRAP_PERIODS)

/**
 * \brief Check if Chaos is synchronized.
 *
 * The application assumes that a node is synchronized if it updated the reference time
 * during the last Chaos phase.
 * \sa \link is_t_ref_l_updated \endlink
 */
#define CHAOS_IS_SYNCED()          (is_t_ref_l_updated())

/**
 * \brief Get Chaos reference time.
 * \sa \link get_t_ref_l \endlink
 */
#define CHAOS_REFERENCE_TIME       (get_t_ref_l())

/** @} */

/**
 * \defgroup Mobashir Variables
 * @{
 */

extern int has_data;
extern unsigned int arrayIndex; 
extern unsigned int arrayOffset;
extern unsigned long curr_seq;

extern unsigned long curr_slot;
extern unsigned long slots;
extern unsigned long ticks;

extern int just_started;

// ebramkw
// duty_cycling
extern uint8_t has_data_update;
extern uint8_t send_data_now;

// ebramkw
// sensing
#define SAMPLING_RATE CLOCK_SECOND * 10 // 30 sec // sampling every ? seconds
#define COMMUNICATION_WAITING CLOCK_SECOND * 120 // 1 m // start another sensing after ? seconds

/** @} */

/** @} */

#endif /* CHAOS_TEST_H_ */
