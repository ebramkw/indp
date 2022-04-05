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
 *
 * \mainpage
 *           These files document the source code of Chaos, a data collection, aggregation, and dissemination system,
 *           for wireless sensor networks (and IoT, M2M, etc.). Chaos is implemented in
 *           <a href="http://www.sics.se/contiki/">Contiki</a> and bases on Tmote Sky sensor nodes.
 *           It uses the
 *           <a href="http://people.ee.ethz.ch/~ferrarif/sw/glossy/">Glossy</a> source code.
 *
 *           Chaos was published at ACM SenSys'13 in the paper titled
 *           <a href="http://www.cse.chalmers.se/~olafl/papers/2013-11-sensys-landsiedel-chaos.pdf">
 *           Chaos: Versatile and Efficient All-to-All Data Sharing and In-Network Processing at Scale</a>.
 *
 *           This documentation is divided into three main parts:
 *           \li \ref chaos-test "Simple application for testing Chaos":
 *           Example of a simple application that periodically floods a packet and prints related statistics.
 *           \li \ref chaos_interface "Chaos API":
 *           API provided by Chaos for an application that wants to use it.
 *           \li \ref chaos_internal "Chaos internal functions":
 *           Functions used internally by Chaos during a round.
 *
 *           A complete overview of the documentation structure is available <a href="modules.html">here</a>.
 *
 * \author
 *           <a href="http://www.cse.chalmers.se/~olafl/">Olaf Landsiedel</a> <olafl@chalmers.se>
 * \author
 *           <a href="http://www.tik.ee.ethz.ch/~ferrarif">Federico Ferrari</a> <ferrari@tik.ee.ethz.ch>
 *
 */

/**
 * \defgroup chaos-test Simple application for testing Chaos
 *
 *           This application runs Chaos periodically.
 *           A round is started by one node (initiator), the application prints statistics at the end of the round.
 *
 *           The application schedules Chaos periodically with a fixed period \link CHAOS_PERIOD \endlink.
 *
 *           The duration of each Chaos phase is given by \link CHAOS_DURATION \endlink.
 *
 *           During each Chaos phase, the maximum number of transmissions in Chaos (N)
 *           is set to \link N_TX \endlink.
 *
 *           The initiator of the floods is the node having nodeId \link INITIATOR_NODE_ID \endlink.
 *
 *           The packet to be flooded has the format specified by data structure \link chaos_data_struct \endlink.
 *
 *           Receivers synchronize by computing the reference time during each Chaos phase.
 *
 *           To synchronize fast, at startup receivers run Chaos with a significantly shorter period
 *           (\link CHAOS_INIT_PERIOD \endlink) and longer duration (\link CHAOS_INIT_DURATION \endlink).
 *
 *           Receivers exit the bootstrapping phase when they have computed the reference time for
 *           \link CHAOS_BOOTSTRAP_PERIODS \endlink consecutive Chaos phases.
 *
 * @{
 */

/**
 * \file
 *         A simple example of an application that uses Chaos, source file.
 *
 * \author
 *         Olaf Landsiedel <olafl@chalmers.se>
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#include "indp-test.h"
#include "chaos.h"
#include "oofg.h" 

/**
 * \defgroup ebramkw headers
 * @{
 */

#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"
#include "svd.h"

#include "dev/light-sensor.h"
#include "dev/sht11/sht11-sensor.h"

/** @} */

/**
 * \defgroup chaos-test-variables Application variables
 * @{
 */

/**
 * \defgroup chaos-test-variables-sched-sync Scheduling and synchronization variables
 * @{
 */

static chaos_data_struct chaos_data;     /**< \brief Flooding data. */
static struct rtimer rt;                   /**< \brief Rtimer used to schedule Chaos. */
static struct pt pt;                       /**< \brief Protothread used to schedule Chaos. */
static rtimer_clock_t t_ref_l_old = 0;     /**< \brief Reference time computed from the Chaos
                                                phase before the last one. \sa get_t_ref_l */
static uint8_t skew_estimated = 0;         /**< \brief Not zero if the clock skew over a period of length

                                                \link CHAOS_PERIOD \endlink has already been estimated. */
static uint8_t sync_missed = 0;            /**< \brief Current number of consecutive phases without
                                                synchronization (reference time not computed). */
static rtimer_clock_t t_start = 0;         /**< \brief Starting time (low-frequency clock)
                                                of the last Chaos phase. */
static int period_skew = 0;                /**< \brief Current estimation of clock skew over a period
                                                of length \link CHAOS_PERIOD \endlink. */

/** @} */

/**
 * \defgroup chaos-test-variables-stats Statistics variables
 * @{
 */

static unsigned long packets_received = 0; /**< \brief Current number of received packets. */
static unsigned long packets_missed = 0;   /**< \brief Current number of missed packets. */
static unsigned long latency = 0;          /**< \brief Latency of last Chaos phase, in us. */
static unsigned long sum_latency = 0;      /**< \brief Current sum of latencies, in ticks of low-frequency
                                                clock (used to compute average). */

/** @} */

/**
 * \defgroup Mobashir Variables
 * @{
 */

unsigned int arrayIndex;
unsigned int arrayOffset;
int has_data;

unsigned long curr_slot;
unsigned long slots;
unsigned long ticks;

#ifndef TESTBED
	#define MERGE 1
#endif
#define JUST_STARTED 6
int just_started;
unsigned long curr_seq = 0;

int curr_payload_len;

static struct etimer et_sensors;

uint8_t has_data_update;

#if CHAOS_SETUP
        uint8_t curr_setup_rounds;
#endif

uint8_t periodic_mode;
uint8_t curr_periodic_rounds;

uint8_t channel_is_free;
uint8_t src_got_answer;

// unsigned short source_node[CHAOS_NODES] = {1, 8, 3, 11, 2, 10, 4, 12, 0, 9, 7, 13, 6, 15, 5, 14};
uint8_t source_node_index;

int cca;
int cca_switched;

uint8_t send_data_now;

unsigned long control_total_secs;

#if ONE_SRC
unsigned long one_total_secs;
#endif

// int cca_log[100];
// uint8_t cca_index;

/** @} */
/** @} */

// uint8_t data_sender;

// phase = 0;
// data_round = 0;

/**
 * \defgroup chaos-test-processes Application processes and functions
 * @{
 */

/**
 * \defgroup chaos-test-print-stats Print statistics information
 * @{
 */

PROCESS(sensing_process, "senseing process");
PROCESS(chaos_print_stats_process, "Chaos print stats");


PROCESS_THREAD(sensing_process, ev, data){
    static int i = 0, val, dec;
    static float s = 0, frac;
    PROCESS_BEGIN();
    while(1){
        #if CHAOS_SETUP
        if(curr_setup_rounds < SETUP_ROUNDS)
            PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
        #endif

        // #if !ONE_SRC
        // if(!has_data_update && !IS_INITIATOR())
        // 	has_data_update = 1;
        // #endif

        etimer_set(&et_sensors, SAMPLING_RATE);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_sensors));

        SENSORS_ACTIVATE(light_sensor);
        SENSORS_ACTIVATE(sht11_sensor);

        // printf("%d: ", i);

        val = sht11_sensor.value(SHT11_SENSOR_TEMP);
        if(val != -1){
                s = ((0.01*val) - 39.60);
                data_matrix[i][0] = s;
        } else{
                data_matrix[i][0] = 0;
        }
        // dec = s;
        // frac = s - dec;
        // printf("%d.%02u ", dec, (unsigned int)(frac * 100));
        
        val = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
        if(val != -1){
                s = (((0.0405*val) - 4) + ((-2.8 * 0.000001)*(pow(val,2))));
                data_matrix[i][1] = s;
        } else{
                data_matrix[i][1] = 0;
        }
        // dec = s;
        // frac = s - dec;
        // printf("%d.%02u ", dec, (unsigned int)(frac * 100));

        val = light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR);
        if(val != -1){
                s = (float)(val * 0.4071);
                data_matrix[i][2] = s;
        } else{
                data_matrix[i][2] = 0;
        }
        // dec = s;
        // frac = s - dec;
        // printf("%d.%02u\n", dec, (unsigned int)(frac * 100));


        SENSORS_DEACTIVATE(light_sensor);
        SENSORS_DEACTIVATE(sht11_sensor);

        i = (i + 1) % M;

        if(i == 0){
            // calculate local pca
            local_svd();

            // compare with previous to decide
            // event
            // need for update or not!

// ********************************************************************************************* //
            // for testing
            // 1 OR all
            #if ONE_SRC
            	if(source_node_index == 0){ // should be source_node_index so different node start comm in different rounds!
            #else
            	if(IS_INITIATOR()){
            		send_data_now = 1;
            #endif
            	// printf("has_data_update\n");
            	has_data_update = 1;
            }
            
            source_node_index = (source_node_index + 1) % CHAOS_NODES;

// ********************************************************************************************* //
            
            // start collecting data again after one minute of communication
            etimer_reset(&et_sensors);
            etimer_set(&et_sensors, COMMUNICATION_WAITING);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_sensors));
        }

        etimer_reset(&et_sensors);
    }
    PROCESS_END();
}

PROCESS_THREAD(chaos_print_stats_process, ev, data)
{
	PROCESS_BEGIN();
	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

                // int j;
                // for(j = 0; j < 100; j++)
                //         printf("%d ", cca_log[j]);
                // printf("\n");

		// leds_on(LEDS_RED);
		// Print statistics only if Chaos is not still bootstrapping.
		if (!CHAOS_IS_BOOTSTRAPPING()) {
			if (get_rx_cnt()) {	// Packet received at least once.
				// Increment number of successfully received packets.
				packets_received++;
				// Compute latency during last Chaos phase.
				rtimer_clock_t lat = get_t_first_rx_l() - get_t_ref_l();
				// Add last latency to sum of latencies.
				sum_latency += lat;
				// Convert latency to microseconds.
				latency = (unsigned long)(lat) * 1e6 / RTIMER_SECOND;
				// Print information about last packet and related latency.
				// printf("seq_no %lu\n", chaos_data.seq_no);
				// printf("data_round %lu\n", chaos_data.data_round);
			} else {	// Packet not received.
				// Increment number of missed packets.
				packets_missed++;
				// Print failed reception.
				// printf("Chaos NOT received\n");
			}
#if CHAOS_DEBUG
//			printf("skew %ld ppm\n", (long)(period_skew * 1e6) / CHAOS_PERIOD);
			// printf("high_T_irq %u, rx_timeout %u, bad_length %u, bad_header %u, bad_crc %u, T_slot_h %u, rc_up %u\n",
			// 		high_T_irq, rx_timeout, bad_length, bad_header, bad_crc, get_T_slot_h(), rc_update);
#endif /* CHAOS_DEBUG */
#if ENERGEST_CONF_ON
			// Compute average radio-on time, in microseconds.
			// unsigned long avg_radio_on = (unsigned long)CHAOS_PERIOD * 1e6 / RTIMER_SECOND *
			// 		(energest_type_time(ENERGEST_TYPE_LISTEN) + energest_type_time(ENERGEST_TYPE_TRANSMIT)) /
			// 		(energest_type_time(ENERGEST_TYPE_CPU) + energest_type_time(ENERGEST_TYPE_LPM));
			// Print information about average radio-on time.
			// printf("average radio-on time %lu.%03lu ms\n",
			// 		avg_radio_on / 1000, avg_radio_on % 1000);
#endif /* ENERGEST_CONF_ON */
		}

#ifdef LOG_TIRQ
		// print_tirq();
#endif
#ifdef LOG_FLAGS
		// print_flags_tx();
		// print_flags_rx();
		// print_stats();
		// print_g();
#endif /* LOG_FLAGS */
		int i;

		// if(CHAOS_IS_BOOTSTRAPPING())
		// 	printf("CHAOS_IS_BOOTSTRAPPING\n");

		// if((IS_INITIATOR() && packets_received > 1) || !IS_INITIATOR()){
		CHAOS_BOOTSTRAPPING = CHAOS_IS_BOOTSTRAPPING();
		if(!CHAOS_IS_BOOTSTRAPPING()){
			if(slots==0&&ticks==0) {
				// if couldn't solve completely
				slots = MERGE;
			}
			unsigned long total_ticks = ((3*slots)*(RTIMER_SECOND/2)) + ticks;
			unsigned long total_secs = (total_ticks*1e3)/RTIMER_SECOND;

			if(control_total_secs == 0)
				control_total_secs = total_secs;

			if(1||num_solved<CHAOS_NODES) {
				// printf("node %d, Solved %d in Elapsed: %3lu.%03lu secs\n", node_id, num_solved, total_secs/1000, total_secs%1000);

				#if R_PCA
					#if LOSSY_RPCA
						printf("node %d, number_not_decoded_data %d, number_not_decoded_data_control %d, no_update(%d):", node_id, number_not_decoded_2, number_not_decoded_3, num_no_update);
					#else
						printf("node %d, no_update(%d):", node_id, num_no_update);
					#endif
					//#if !TOTAL_NODES_RUN
					#if TOTAL_NODES_RUN
						for(i=0;i<TOTAL_NODES;i++) {
					#else
						for(i=0;i<CHAOS_NODES;i++) {
					#endif
						printf("%d ", no_update[i]);
					}
					#if LOSSY_RPCA
						printf(", data(%d):", num_data);
						#if TOTAL_NODES_RUN
							for(i=0;i<TOTAL_NODES;i++) {
						#else
							for(i=0;i<CHAOS_NODES;i++) {
						#endif
							printf("%d ", data_control[i]);
						}
						printf(", actual_num_no_update %d, actual_num_solved %d", actual_num_no_update, actual_num_solved);
						
						printf(", reliability_control ");
						reliability[NUM_RECORDS - 1] = num_data + num_no_update;
						printf("%d ", reliability[0]);
						for (i = 1; i < NUM_RECORDS; i++){
							if(reliability[i] == 100)
								reliability[i] = reliability[i - 1];
							printf("%d ", reliability[i]);
						}

						printf(", reliability_solved ");
						reliability_solved[NUM_RECORDS - 1] = num_solved;
						printf("%d ", reliability_solved[0]);
						for (i = 1; i < NUM_RECORDS; i++){
							if(reliability_solved[i] == 100)
								reliability_solved[i] = reliability_solved[i - 1];
							printf("%d ", reliability_solved[i]);
						}
						
						printf(", reliability_time ");
						for (i = 0; i < NUM_RECORDS; i++){
							printf("%d ", reliability_time[i]);
						}
					#endif
					printf(", Solved %d,", num_solved);
				#else
					printf("node %d, Solved(%d),", node_id, num_solved);
				#endif
				// for(i=0;i<num_solved;i++) {
				// 	printf("%d ", solved[i]+1);
				// }
				if((num_no_update + num_data) == CHAOS_NODES)
					printf(" in Control_Elapsed, %3lu.%03lu, secs,", control_total_secs/1000, control_total_secs%1000);
				else
					printf(" in Control_Elapsed, , , secs,");

				#if ONE_SRC
				if(num_solved == 1){
					if(one_total_secs == 100)
						one_total_secs = 0;
					printf(" one_node_Solved_Elapsed, %3lu.%03lu, secs,", one_total_secs/1000, one_total_secs%1000);
				#else
				if(num_solved == CHAOS_NODES){
				#endif
					printf(" total_nodes_Solved_Elapsed, %3lu.%03lu, secs. ", total_secs/1000, total_secs%1000);
				}
				else
					printf(" Solved_Elapsed, , , secs. ");
				printf("seq_no %lu\n", chaos_data.seq_no);

				// printf("num_solved: ");
				// for(i = 0; i < 100; i++)
				// 	printf("%lu ", num_solved_log[i]);
				// printf("\n");
			}

			// latency
			// printf("node %d, Elapsed: %3lu.%03lu secs \n", node_id, total_secs/1000, total_secs%1000);

			// compute global pca
			
			global_pca();
			// printf("computed global PCA\n");

			// ebramkw: print Y matrix
			// print_g();
		}

		// rtimer_clock_t emax = 0;
		// rtimer_clock_t dmax = 0;
		// printf("DP1(%d): ",dp_count);
		// for(i=0;i<dp_count;i++) {
		// 	if(decode_time_log[i]>emax)
		// 		dmax = decode_time_log[i];
		// 	printf("%u ",decode_time_log[i]);
		// }
		// printf("\n");
		// printf("DP2(%d): ",dp_count);
		// for(i=0;i<dp_count;i++) {
		// 	if(encode_time_log[i]>emax)
		// 		emax = encode_time_log[i];
		// 	printf("%u ",encode_time_log[i]);
		// }
		// printf("\n");
		// printf("MAX: Encode %u Decode %u Sum %u\n", emax, dmax, emax+dmax);
		// // dp_count = 0;

		// printf("DP(%d): ",dp_count);
		// rtimer_clock_t max_ofg = 0;
		// for(i=0;i<dp_count;i++) {
		// 	rtimer_clock_t temp_ofg = encode_time_log[i]+decode_time_log[i];
		// 	if(temp_ofg>max_ofg)
		// 		max_ofg = temp_ofg;
		// 	// printf("%u ",temp_ofg);
		// }
		// // printf("\n");
		// printf("max_ofg(%d): %u\n", dp_count, max_ofg);

// NB: tx_to_nbr, rx_from_nbr, rxn_from_nbr are all actual node ID and not

// #if RLNC == 1
// 		printf("tx_activity(%d): ",tx_activity_cnt);
// 		for(i=0;i<tx_activity_cnt;i++) {
// 			printf("%d-%d ",tx_activity[i],tx_to_nbr[i][0]);
// 		}
// 		printf("\n");
// 		printf("rxn_activity(%d): ",rxn_activity_cnt);
// 		for(i=0;i<rxn_activity_cnt;i++) {
// 			printf("%d-%d ",rxn_activity[i],rxn_from_nbr[i][0]);
// 		}
// 		printf("\n");
// 		printf("rx_activity(%d): ",rx_activity_cnt);
// 		for(i=0;i<rx_activity_cnt;i++) {
// 			printf("%d-%d ",rx_activity[i],rx_from_nbr[i][0]);
// 		}
// 		printf("\n");
// #elif RLNC == 2
// 		printf("tx_activity(%d): ",tx_activity_cnt);
// 		for(i=0;i<tx_activity_cnt;i++) {
// 			printf("%d-%d-%d ",tx_activity[i],tx_to_nbr[i][0],tx_to_nbr[i][1]);
// 		}
// 		printf("\n");
// 		printf("rxn_activity(%d): ",rxn_activity_cnt);
// 		for(i=0;i<rxn_activity_cnt;i++)
// 			printf("%d ",rxn_activity[i]);
// 		printf("\n");
// 		printf("rx_activity(%d): ",rx_activity_cnt);
// 		for(i=0;i<rx_activity_cnt;i++)
// 			printf("%d ",rx_activity[i]);
// 		printf("\n");
// #endif

		// leds_off(LEDS_RED);
	}
	PROCESS_END();
}

/** @} */

/**
 * \defgroup chaos-test-skew Clock skew estimation
 * @{
 */

static inline void estimate_period_skew(void) {
	// Estimate clock skew over a period only if the reference time has been updated.
	if (CHAOS_IS_SYNCED()) {
		// Estimate clock skew based on previous reference time and the Chaos period.
		period_skew = get_t_ref_l() - (t_ref_l_old + (rtimer_clock_t)CHAOS_PERIOD);
		// Update old reference time with the newer one.
		t_ref_l_old = get_t_ref_l();
		// If Chaos is still bootstrapping, count the number of consecutive updates of the reference time.
		if (CHAOS_IS_BOOTSTRAPPING()) {
			// Increment number of consecutive updates of the reference time.
			skew_estimated++;
			// Check if Chaos has exited from bootstrapping.
			if (!CHAOS_IS_BOOTSTRAPPING()) {
				// Chaos has exited from bootstrapping.
				//leds_off(LEDS_RED);
				// Initialize Energest values.
				energest_init();
#if CHAOS_DEBUG
				high_T_irq = 0;
				bad_crc = 0;
				bad_length = 0;
				bad_header = 0;
#endif /* CHAOS_DEBUG */

			}
		}
	}
}

/** @} */

/**
 * \defgroup chaos-test-scheduler Periodic scheduling
 * @{
 */

uint16_t node_index;

static inline void setArrayIndex(void){
	//all flags to zero
	// memset(&chaos_data.flags[0], 0, MERGE_LEN * sizeof(uint8_t));
	//all bv to zero
	memset(&chaos_data.bv[0], 0, MERGE_LEN * sizeof(uint8_t));
	//find my index and offset
	// arrayIndex = node_index / 8;
	// arrayOffset = node_index % 8;
	//set to one at index and offset
  	// chaos_data.bv[arrayIndex] = 1 << arrayOffset;

  	memset(&chaos_data.solved_bv[0], 0, MERGE_LEN * sizeof(uint8_t));

  	// uint8_t i;
  	// for(i = 0; i < MERGE_LEN; i++){
  	// 	chaos_data.no_update[i] = 0;
  	// 	chaos_data.data_control[i] = 0;
  	// 	chaos_data.bv[i] = 0;
  	// 	chaos_data.solved_bv[i] = 0;
  	// }
  	memset(&chaos_data.no_update[0], 0, MERGE_LEN * sizeof(uint8_t));
  	memset(&chaos_data.data_control[0], 0, MERGE_LEN * sizeof(uint8_t));

 	// #if TOTAL_NODES_RUN
	//   	#if R_PCA
	// 		memset(&chaos_data.no_update[0], 0, TOTAL_MERGE_LEN * sizeof(uint8_t));
	// 		#if LOSSY_RPCA
	// 			memset(&chaos_data.data_control[0], 0, TOTAL_MERGE_LEN * sizeof(uint8_t));
	// 		#endif
	// 	#endif
	// #else
	// 	#if R_PCA
	// 		memset(&chaos_data.no_update[0], 0, MERGE_LEN * sizeof(uint8_t));
	// 		#if LOSSY_RPCA
	// 			memset(&chaos_data.data_control[0], 0, MERGE_LEN * sizeof(uint8_t));
	// 		#endif
	// 	#endif
	// #endif

  	return;
}
	
static inline void setData(){
	// ebramkw
	int i, j, k;
	char data_bytes[4];
	
	memset(&chaos_data.payload[0], 0, PAYLOAD_LEN * sizeof(char));

    if(has_data_update){
        // insert eigenvalue
        float2Bytes(singular_value, data_bytes);
        for(j = 0; j < 4; j++)
        	chaos_data.payload[j] = data_bytes[j];
        // insert corresponding eigenvector
        for(k = 1; k < (N + 1); k++){
        	float2Bytes(singular_vector[k - 1], data_bytes);
        	for(j = 0; j < 4; j++)
        		chaos_data.payload[(((k - 1) * 4)+(j + 4))] = data_bytes[j];
        }

        // chaos_data.data_control[node_index/8] |= (1 << (node_index % 8));
        // chaos_data.no_update[node_index/8] &= (0 << (node_index % 8));
    } else{
    	;
        chaos_data.no_update[node_index/8] |= (1 << (node_index % 8));
        // chaos_data.data_control[node_index/8] &= (0 << (node_index % 8));
    }
	
	// set data to eigenvectors and eigenvalues if data_flag = 0
	// insert eigenvalue
	// float2Bytes(singular_value, data_bytes);
	// for(j = 0; j < 4; j++)
	// 	chaos_data.payload[j] = data_bytes[j];
	// insert corresponding eigenvector
	// for(k = 1; k < (N + 1); k++){
	// 	float2Bytes(singular_vector[k - 1], data_bytes);
	// 	for(j = 0; j < 4; j++)
	// 		chaos_data.payload[(((k - 1) * 4)+(j + 4))] = data_bytes[j];
	// }
	
	// set data to data
	// printf("node %d: setdata round %d\n", node_id, data_round);
	// set data to projected data based on data_round
	// 2 (size of data / size of payload) rounds each of 5 floats
	// for(i = 0; i < PAYLOAD_LEN/4; i++){
	// 	float2Bytes(total_projected_data_matrix[node_index][i + (data_round * ((PAYLOAD_LEN/4)))][0], data_bytes);
	// 	for(j = 0; j < 4; j++)
	// 		chaos_data.payload[(j + (i * 4))] = data_bytes[j];
	// }

	// // ebramkw
	// //set data to node id
	// uint8_t i;
	// float d = node_id + 0.5367;
	// char d_bytes[4];
	// float2Bytes(d, d_bytes);
	// for(i = 0; i < PAYLOAD_LEN; i++){
	// 	chaos_data.payload[i] = d_bytes[i%4];
	// }

	//set dummy payload
	// uint8_t i;
	// for( i=0; i < PAYLOAD_LEN; i++ ){
	// 	chaos_data.payload[i] = (uint8_t) (0x11 * i);
	// }
	//set xor payload
	// chaos_data.payload[0] = (uint8_t) X[node_index];
	return;
}

char chaos_scheduler(struct rtimer *t, void *ptr) {
	static uint8_t i = 0;
	PT_BEGIN(&pt);
	// leds_off(LEDS_GREEN);
	// leds_off(LEDS_BLUE);
	// leds_off(LEDS_RED);
	if (has_data) {	// Chaos src.
		while(1){
			// old style mode - setup phase ==> periodic old codecast with initiator and others contribute their data
			#if CHAOS_SETUP
			if((curr_setup_rounds < SETUP_ROUNDS) || periodic_mode){
			#else
			if(periodic_mode){
			#endif
				// old style mode
				// run other sources than initiator as receivers
                if(IS_INITIATOR()){
                    chaos_data.seq_no++;
                    curr_seq = chaos_data.seq_no - 1;

                    // so hopefully all started chaos
                    rtimer_set(t, RTIMER_TIME(t) + SRC_RETRANSMISSION_OFF_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                    PT_YIELD(&pt);

                    //set my flag to one
                    setArrayIndex();
                    //set data
                    setData();
                    ofge_init(node_index);
                    encode(chaos_data.bv, chaos_data.payload);

                    channel_is_free = 1;

                    // Initialise packet parameters
                    chaos_data.source = node_id;
                    // Start Chaos.
                    just_started = JUST_STARTED;
                    chaos_start((uint8_t *)&chaos_data, /*DATA_LEN,*/ CHAOS_INITIATOR, /*CHAOS_SYNC,*/ N_TX);
                    // Store time at which Chaos has started.
                    t_start = RTIMER_TIME(t);

                    #if CHANNEL_HOPPING
                    for(i = 0; i < NUM_OF_100_DURATION; i++){
                        channel_is_free = 1;
                        rtimer_set(t, CHAOS_REFERENCE_TIME + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                        // Yield the protothread.
                        PT_YIELD(&pt);

                        if(channel_is_free){
                            cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                            curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                        }
                    }
                    #else
                    // Schedule end of Chaos phase based on CHAOS_DURATION.
                    rtimer_set(t, CHAOS_REFERENCE_TIME + CHAOS_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                    // Yield the protothread.
                    PT_YIELD(&pt);
                    #endif
                    
                    chaos_stop();

                    #if SETUP_ROUNDS
                    if(curr_setup_rounds < SETUP_ROUNDS)
                    	curr_setup_rounds++;
                    #endif


                    if (!CHAOS_IS_BOOTSTRAPPING()) {
                        // Chaos has already successfully bootstrapped.
                        if (!CHAOS_IS_SYNCED()) {
                            // The reference time was not updated: increment reference time by CHAOS_PERIOD.
                            set_t_ref_l(CHAOS_REFERENCE_TIME + CHAOS_PERIOD);
                            set_t_ref_l_updated(1);
                        }
                    }

                    if(periodic_mode){
                        if((num_no_update + num_solved) == CHAOS_NODES){
                            periodic_mode = 0;
                            has_data_update = 0;
                        }
                        else
                            periodic_mode = 1;
                    }

                    // periodic_mode up to MAX_PERIODIC_ROUNDS
                    // if(periodic_mode){
                    //         if((curr_periodic_rounds + 1) == MAX_PERIODIC_ROUNDS){
                    //                 curr_periodic_rounds = 0;
                    //                 periodic_mode = 0;

                    //                 has_data_update = 0;
                    //         } else{
                    //                 curr_periodic_rounds++;
                    //         }
                    // } else{
                    //         curr_periodic_rounds = 0;
                    // }

                    // run periodic_mode until no more data
                    if(periodic_mode){
                        if(num_solved == 0)
                            periodic_mode = 0;
                    }

                    // Schedule begin of next Chaos phase based on CHAOS_PERIOD.
                    rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD, (rtimer_callback_t)chaos_scheduler, ptr);
                    // Poll the process that prints statistics (will be activated later by Contiki).
                    process_poll(&chaos_print_stats_process);
                    #if CHAOS_SETUP
                    if(curr_setup_rounds >= SETUP_ROUNDS)
                        process_poll(&sensing_process);
                    #endif
                    // Yield the protothread.
                    PT_YIELD(&pt);
                    // Estimate the clock skew over the last period.
                    estimate_period_skew();
                } else{
                    channel_is_free = 1;

                    setArrayIndex();
                    ofge_init(node_index);
                    just_started = JUST_STARTED;
                    chaos_start((uint8_t *)&chaos_data, /*DATA_LEN,*/ CHAOS_RECEIVER, /*CHAOS_SYNC,*/ N_TX);
                    t_start = RTIMER_TIME(t);

                    if (CHAOS_IS_BOOTSTRAPPING()) {
                        // Chaos is still bootstrapping:
                        // Schedule end of Chaos phase based on CHAOS_INIT_DURATION.
                        // MM: Duration+49ms
                        #if CHANNEL_HOPPING
                        for(i = 0; i < NUM_OF_100_DURATION; i++){
                            channel_is_free = 1;
                            rtimer_set(t,  RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                            // Yield the protothread.
                            PT_YIELD(&pt);

                            if(channel_is_free){
                                cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                                curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                            }
                        }
                        rtimer_set(t, RTIMER_TIME(t) + (CHAOS_INIT_GUARD_TIME - CHAOS_GUARD_TIME), 1,
                        (rtimer_callback_t)chaos_scheduler, ptr);
                        #else
                        rtimer_set(t, RTIMER_TIME(t) + CHAOS_INIT_DURATION, 1,
                        (rtimer_callback_t)chaos_scheduler, ptr);
                        #endif
                    } else {
                        // Chaos has already successfully bootstrapped:
                        // Schedule end of Chaos phase based on CHAOS_DURATION.
                        // MM: Duration+(1+syncmissed) ms
                        #if CHANNEL_HOPPING
                        for(i = 0; i < NUM_OF_100_DURATION; i++){
                            channel_is_free = 1;
                            rtimer_set(t,  RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                            // Yield the protothread.
                            PT_YIELD(&pt);

                            if(channel_is_free){
                                cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                                curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                            }
                        }
                        rtimer_set(t, RTIMER_TIME(t) + CHAOS_GUARD_TIME * (1 + sync_missed), 1,
                        (rtimer_callback_t)chaos_scheduler, ptr);
                        #else
                        rtimer_set(t, RTIMER_TIME(t) + CHAOS_GUARD_TIME * (1 + sync_missed) + CHAOS_DURATION, 1,
                        (rtimer_callback_t)chaos_scheduler, ptr);
                        #endif
                    }
                    // Yield the protothread.
                    PT_YIELD(&pt);

                    chaos_stop();

                    #if SETUP_ROUNDS
                    if(curr_setup_rounds < SETUP_ROUNDS)
                    	curr_setup_rounds++;
                    #endif

                    if (CHAOS_IS_BOOTSTRAPPING()) {
                        // Chaos is still bootstrapping.
                        if (!CHAOS_IS_SYNCED()) {
                            // The reference time was not updated: reset skew_estimated to zero.
                            skew_estimated = 0;
                        }
                    } else {
                        // Chaos has already successfully bootstrapped.
                        if (!CHAOS_IS_SYNCED()) {
                            // The reference time was not updated:
                            // increment reference time by CHAOS_PERIOD + period_skew.
                            set_t_ref_l(CHAOS_REFERENCE_TIME + CHAOS_PERIOD + period_skew);
                            set_t_ref_l_updated(1);
                            // Increment sync_missed.
                            sync_missed++;
                        } else {
                            // The reference time was not updated: reset sync_missed to zero.
                            sync_missed = 0;
                        }

                        // Poll the process that prints statistics (will be activated later by Contiki).
                        process_poll(&chaos_print_stats_process);
                    }

                    if(periodic_mode){
                        if((num_no_update + num_solved) == CHAOS_NODES){
                            periodic_mode = 0;
                            has_data_update = 0;
                        }
                        else
                            periodic_mode = 1;
                    }

                    // periodic_mode up to MAX_PERIODIC_ROUNDS
                    // if(periodic_mode){
                    //         if((curr_periodic_rounds + 1) == MAX_PERIODIC_ROUNDS){
                    //                 curr_periodic_rounds = 0;
                    //                 periodic_mode = 0;

                    //                 has_data_update = 0;
                    //         } else{
                    //                 curr_periodic_rounds++;
                    //         }
                    // } else{
                    //         curr_periodic_rounds = 0;
                    // }

                    // run periodic_mode until no more data
                    if(periodic_mode){
                        if(num_solved == 0)
                            periodic_mode = 0;
                    }


                    #if CHAOS_SETUP
                    if(curr_setup_rounds >= SETUP_ROUNDS)
                        process_poll(&sensing_process);
                    #endif

                    // Estimate the clock skew over the last period.
                    estimate_period_skew();

                    // help sync  it
                    if (CHAOS_IS_BOOTSTRAPPING()) {
                        // Chaos is still bootstrapping.
                        if (skew_estimated == 0) {
                            // The reference time was not updated:
                            // Schedule begin of next Chaos phase based on last begin and CHAOS_INIT_PERIOD.
                            // MM: 10 ms
                            rtimer_set(t, RTIMER_TIME(t) + CHAOS_INIT_PERIOD - CHAOS_INIT_DURATION, 1,
                                            (rtimer_callback_t)chaos_scheduler, ptr);
                        } else {
                            // The reference time was updated:
                            // Schedule begin of next Chaos phase based on reference time and CHAOS_INIT_PERIOD.
                            // MM: (Period-20) ms
                            rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD - CHAOS_INIT_GUARD_TIME,
                                            (rtimer_callback_t)chaos_scheduler, ptr);
                        }
                    } else {
                        // Chaos has already successfully bootstrapped:
                        // Schedule begin of next Chaos phase based on reference time and CHAOS_PERIOD.
                        // MM: Period-(1+syncmissed) ms
                        rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD +
                                        period_skew - CHAOS_GUARD_TIME * (1 + sync_missed),
                                        (rtimer_callback_t)chaos_scheduler, ptr);
                    }
                    // Yield the protothread.
                    PT_YIELD(&pt);
                }
                // fixed time at the end
                rtimer_set_long(t, RTIMER_TIME(t) + FIXED_RADIO_OFF_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                PT_YIELD(&pt);
        	} else {
        		#if ONE_SRC
                // if has data to share, start as initiator - else sample the channel
                if(has_data_update){ // wake-up network and send my data
                #else
                // 16 srcs
                if(send_data_now && IS_INITIATOR()){ // wake-up network and send my data
                #endif
                    // if I have data, try to send it
                    chaos_data.seq_no++;
                    curr_seq = chaos_data.seq_no - 1;

                    for(i = 0; i < WAKE_UP_NETWORK_NUM; i++){
                        // change channel every two rounds
                        #if CHANNEL_HOPPING
                        cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                        curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                        #endif

                        src_got_answer = 0;
                        // start chaos to send packet
                        //set my flag to one
                        setArrayIndex();
                        //set data
                        setData();
                        ofge_init(node_index);
                        encode(chaos_data.bv, chaos_data.payload);

                        // Initialise packet parameters
                        chaos_data.source = node_id;
                        // Start Chaos.
                        just_started = JUST_STARTED;
                        chaos_start((uint8_t *)&chaos_data, /*DATA_LEN,*/ CHAOS_INITIATOR, /*CHAOS_SYNC,*/ N_TX);
                        // Store time at which Chaos has started.
                        t_start = RTIMER_TIME(t);

                        // set a timer and check if someone answer to me
                        rtimer_set(t, RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                        PT_YIELD(&pt);
                        if(src_got_answer){
                            // success to wake-up another node
                            break;
                        } else{
                            // stop and try again later
                            chaos_stop();
                            rtimer_set(t, RTIMER_TIME(t) + SRC_RETRANSMISSION_OFF_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                            PT_YIELD(&pt);
                        }
                    }

                    if(src_got_answer){
                        #if CHANNEL_HOPPING
                        for(i = 0; i < NUM_OF_100_DURATION; i++){
                            channel_is_free = 1;
                            rtimer_set(t, CHAOS_REFERENCE_TIME + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                            // Yield the protothread.
                            PT_YIELD(&pt);

                            if(channel_is_free){
                                cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                                curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                            }
                        }
                        #else

                        // Schedule end of Chaos phase based on CHAOS_DURATION.
                        rtimer_set(t, CHAOS_REFERENCE_TIME + CHAOS_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                        // Yield the protothread.
                        PT_YIELD(&pt);
                        #endif
                        
                        chaos_stop();
                        if (!CHAOS_IS_BOOTSTRAPPING()) {
                            // Chaos has already successfully bootstrapped.
                            if (!CHAOS_IS_SYNCED()) {
                                // The reference time was not updated: increment reference time by CHAOS_PERIOD.
                                set_t_ref_l(CHAOS_REFERENCE_TIME + CHAOS_PERIOD);
                                set_t_ref_l_updated(1);
                            }
                        }

                        #if ONE_SRC
                        	has_data_update = 0;
                        #else
                        	send_data_now = 0;
                        #endif

                        // dual mode
                        // update periodic_mode
                        // if((num_no_update + num_solved) == CHAOS_NODES){
                        //         periodic_mode = 0;
                        //         has_data_update = 0;
                        // } else{
                        //         periodic_mode = 1;
                        // }

                        rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD, (rtimer_callback_t)chaos_scheduler, ptr);
                        // Poll the process that prints statistics (will be activated later by Contiki).
                        process_poll(&chaos_print_stats_process);
                        // Yield the protothread.
                        PT_YIELD(&pt);
                        // Estimate the clock skew over the last period.
                        estimate_period_skew();

                        // // fixed time at the end
                        rtimer_set_long(t, RTIMER_TIME(t) + FIXED_RADIO_OFF_DURATION, 1,
                        (rtimer_callback_t)chaos_scheduler, ptr);
                        PT_YIELD(&pt);
                    } else{
                    	#if ONE_SRC
                        	has_data_update = 0;
                        #else
                        	send_data_now = 0;
                        #endif
                    }
                } else{ // chaos duty cycling - act as receivers and relays
                    // sampling duty cycling
                    // change channel every two rounds
                    #if CHANNEL_HOPPING
                    if(switched % 2 == 0){
                        cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                        curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                    }
                    switched++;
                    #endif

                    cc2420_radio_on();

                    // sleep for 0.2 ms
                    rtimer_set(t, RTIMER_TIME(t) + SAMPLING_RADIO_ON_DURATION, 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    PT_YIELD(&pt);

                    cca = cc2420_cca();

                    cc2420_radio_off();

                    if(!cca){
                        // has_data but does not has data update
                        channel_is_free = 1;
                        setArrayIndex();
                        ofge_init(node_index);
                        just_started = JUST_STARTED;
                        chaos_start((uint8_t *)&chaos_data, /*DATA_LEN,*/ CHAOS_RECEIVER, /*CHAOS_SYNC,*/ N_TX);
                        t_start = RTIMER_TIME(t);

                        // wait for 50ms and check if something is received
                        rtimer_set(t, RTIMER_TIME(t) + RADIO_ON_DURATION, 1,
                        (rtimer_callback_t)chaos_scheduler, ptr);
                        PT_YIELD(&pt);

                        // if something is received, break
                        // else, stop_chaos now, wait for 100ms and start again
                        if(!channel_is_free){
                            if (CHAOS_IS_BOOTSTRAPPING()) {
                                // Chaos is still bootstrapping:
                                // Schedule end of Chaos phase based on CHAOS_INIT_DURATION.
                                // MM: Duration+49ms
                                #if CHANNEL_HOPPING
                                for(i = 0; i < NUM_OF_100_DURATION; i++){
                                    channel_is_free = 1;
                                    rtimer_set(t,  RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                                    // Yield the protothread.
                                    PT_YIELD(&pt);

                                    if(channel_is_free){
                                        cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                                        curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                                    }
                                }
                                rtimer_set(t, RTIMER_TIME(t) + (CHAOS_INIT_GUARD_TIME - CHAOS_GUARD_TIME), 1,
                                (rtimer_callback_t)chaos_scheduler, ptr);
                                #else
                                rtimer_set(t, RTIMER_TIME(t) + CHAOS_INIT_DURATION, 1,
                                (rtimer_callback_t)chaos_scheduler, ptr);
                                #endif
                            } else {
                                // Chaos has already successfully bootstrapped:
                                // Schedule end of Chaos phase based on CHAOS_DURATION.
                                // MM: Duration+(1+syncmissed) ms
                                #if CHANNEL_HOPPING
                                for(i = 0; i < NUM_OF_100_DURATION; i++){
                                    channel_is_free = 1;
                                    rtimer_set(t,  RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                                    // Yield the protothread.
                                    PT_YIELD(&pt);

                                    if(channel_is_free){
                                        cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                                        curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                                    }
                                }
                                rtimer_set(t, RTIMER_TIME(t) + + CHAOS_GUARD_TIME * (1 + sync_missed), 1,
                                (rtimer_callback_t)chaos_scheduler, ptr);
                                #else
                                rtimer_set(t, RTIMER_TIME(t) + CHAOS_GUARD_TIME * (1 + sync_missed) + CHAOS_DURATION, 1,
                                (rtimer_callback_t)chaos_scheduler, ptr);
                                #endif
                            }
                            // Yield the protothread.
                            PT_YIELD(&pt);

                            chaos_stop();

                            if (CHAOS_IS_BOOTSTRAPPING()) {
                                // Chaos is still bootstrapping.
                                if (!CHAOS_IS_SYNCED()) {
                                    // The reference time was not updated: reset skew_estimated to zero.
                                    skew_estimated = 0;
                                }
                            } else {
                                // Chaos has already successfully bootstrapped.
                                if (!CHAOS_IS_SYNCED()) {
                                    // The reference time was not updated:
                                    // increment reference time by CHAOS_PERIOD + period_skew.
                                    set_t_ref_l(CHAOS_REFERENCE_TIME + CHAOS_PERIOD + period_skew);
                                    set_t_ref_l_updated(1);
                                    // Increment sync_missed.
                                    sync_missed++;
                                } else {
                                    // The reference time was not updated: reset sync_missed to zero.
                                    sync_missed = 0;
                                }

                                // Poll the process that prints statistics (will be activated later by Contiki).
                                process_poll(&chaos_print_stats_process);
                            }

                            // update periodic_mode
                            // if((num_no_update + num_solved) == CHAOS_NODES)
                            //         periodic_mode = 0;
                            // else
                            //         periodic_mode = 1;

                            // Estimate the clock skew over the last period.
                            estimate_period_skew();

                            // help sync  it
                            if (CHAOS_IS_BOOTSTRAPPING()) {
                                // Chaos is still bootstrapping.
                                if (skew_estimated == 0) {
                                    // The reference time was not updated:
                                    // Schedule begin of next Chaos phase based on last begin and CHAOS_INIT_PERIOD.
                                    // MM: 10 ms
                                    rtimer_set(t, RTIMER_TIME(t) + CHAOS_INIT_PERIOD - CHAOS_INIT_DURATION, 1,
                                                    (rtimer_callback_t)chaos_scheduler, ptr);
                                } else {
                                    // The reference time was updated:
                                    // Schedule begin of next Chaos phase based on reference time and CHAOS_INIT_PERIOD.
                                    // MM: (Period-20) ms
                                    rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD - CHAOS_INIT_GUARD_TIME,
                                                    (rtimer_callback_t)chaos_scheduler, ptr);
                                }
                            } else {
                                // Chaos has already successfully bootstrapped:
                                // Schedule begin of next Chaos phase based on reference time and CHAOS_PERIOD.
                                // MM: Period-(1+syncmissed) ms
                                rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD +
                                                period_skew - CHAOS_GUARD_TIME * (1 + sync_missed),
                                                (rtimer_callback_t)chaos_scheduler, ptr);
                            }
                            // Yield the protothread.
                            PT_YIELD(&pt);
                            // fixed time at the end
                            rtimer_set_long(t, RTIMER_TIME(t) + FIXED_RADIO_OFF_DURATION, 1,
                            (rtimer_callback_t)chaos_scheduler, ptr);
                            PT_YIELD(&pt);
                        } else{
                            chaos_stop();
                        }
                    }
                }

                // time before sampling the channel
                if(cca_switched % NUM_OF_SAMPLING == 0){
                	// sleep for 100 ms
                    rtimer_set(t, RTIMER_TIME(t) + SAMPLING_RADIO_LONG_OFF_DURATION, 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    PT_YIELD(&pt);
                } else{
                	// sleep for 1 ms
                    rtimer_set(t, RTIMER_TIME(t) + SAMPLING_RADIO_OFF_DURATION, 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    PT_YIELD(&pt);
                }
                cca_switched++;

                // rtimer_set(t, RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                // PT_YIELD(&pt);
            }
        }
	} else {	// Chaos receiver.
		while (1) {
            // setup rounds or old style mode ==> periodic
            #if CHAOS_SETUP
            if((curr_setup_rounds < SETUP_ROUNDS) || periodic_mode){
            #else
            if(periodic_mode){
            #endif
                // set data and ofge init is different in different phases
                //set my flag to one
                setArrayIndex();
                ofge_init(node_index);

                channel_is_free = 1;
                
                // Start Chaos.
                just_started = JUST_STARTED;
                chaos_start((uint8_t *)&chaos_data, /*DATA_LEN,*/ CHAOS_RECEIVER, /*CHAOS_SYNC,*/ N_TX);
                // Store time at which Chaos has started.
                t_start = RTIMER_TIME(t);
                if (CHAOS_IS_BOOTSTRAPPING()) {
                    // Chaos is still bootstrapping:
                    // Schedule end of Chaos phase based on CHAOS_INIT_DURATION.
                    // MM: Duration+49ms
                    #if CHANNEL_HOPPING
                    for(i = 0; i < NUM_OF_100_DURATION; i++){
                        channel_is_free = 1;
                        rtimer_set(t,  RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                        // Yield the protothread.
                        PT_YIELD(&pt);

                        if(channel_is_free){
                                cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                                curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                        }
                    }
                    rtimer_set(t, RTIMER_TIME(t) + (CHAOS_INIT_GUARD_TIME - CHAOS_GUARD_TIME), 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    #else
                    rtimer_set(t, RTIMER_TIME(t) + CHAOS_INIT_DURATION, 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    #endif
                } else {
                    // Chaos has already successfully bootstrapped:
                    // Schedule end of Chaos phase based on CHAOS_DURATION.
                    // MM: Duration+(1+syncmissed) ms
                    #if CHANNEL_HOPPING
                    for(i = 0; i < NUM_OF_100_DURATION; i++){
                        channel_is_free = 1;
                        rtimer_set(t,  RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                        // Yield the protothread.
                        PT_YIELD(&pt);

                        if(channel_is_free){
                                cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                                curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                        }
                    }
                    rtimer_set(t, RTIMER_TIME(t) + + CHAOS_GUARD_TIME * (1 + sync_missed), 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    #else
                    rtimer_set(t, RTIMER_TIME(t) + CHAOS_GUARD_TIME * (1 + sync_missed) + CHAOS_DURATION, 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    #endif
                }
                // Yield the protothread.
                PT_YIELD(&pt);

                // MM: Increment slot counter
                // curr_slot++;
                // Stop Chaos.
                chaos_stop();

                #if SETUP_ROUNDS
                if(curr_setup_rounds < SETUP_ROUNDS)
                    curr_setup_rounds++;
                #endif
                
                if (CHAOS_IS_BOOTSTRAPPING()) {
                    // Chaos is still bootstrapping.
                    if (!CHAOS_IS_SYNCED()) {
                        // The reference time was not updated: reset skew_estimated to zero.
                        skew_estimated = 0;
                    }
                } else {
                    // Chaos has already successfully bootstrapped.
                    if (!CHAOS_IS_SYNCED()) {
                        // The reference time was not updated:
                        // increment reference time by CHAOS_PERIOD + period_skew.
                        set_t_ref_l(CHAOS_REFERENCE_TIME + CHAOS_PERIOD + period_skew);
                        set_t_ref_l_updated(1);
                        // Increment sync_missed.
                        sync_missed++;
                    } else {
                        // The reference time was not updated: reset sync_missed to zero.
                        sync_missed = 0;
                    }
                    // Poll the process that prints statistics (will be activated later by Contiki).
                    process_poll(&chaos_print_stats_process);
                }


                // update periodic_mode
                if(periodic_mode){
                    if((num_no_update + num_solved) == CHAOS_NODES)
                        periodic_mode = 0;
                    else
                        periodic_mode = 1;
                }

                // periodic_mode up to MAX_PERIODIC_ROUNDS
                // if(periodic_mode){
                //         if((curr_periodic_rounds + 1) == MAX_PERIODIC_ROUNDS){
                //                 curr_periodic_rounds = 0;
                //                 periodic_mode = 0;
                //         } else{
                //                 curr_periodic_rounds++;
                //         }
                // } else{
                //         curr_periodic_rounds = 0;
                // }

                // run periodic_mode until no more data
                if(periodic_mode){
                    if(num_solved == 0)
                        periodic_mode = 0;
                }

                #if CHAOS_SETUP
                if(curr_setup_rounds >= SETUP_ROUNDS)
                    process_poll(&sensing_process);
                #endif

                // Estimate the clock skew over the last period.
                estimate_period_skew();
                if (CHAOS_IS_BOOTSTRAPPING()) {
                    // Chaos is still bootstrapping.
                    if (skew_estimated == 0) {
                        // The reference time was not updated:
                        // Schedule begin of next Chaos phase based on last begin and CHAOS_INIT_PERIOD.
                        // MM: 10 ms
                        rtimer_set(t, RTIMER_TIME(t) + CHAOS_INIT_PERIOD - CHAOS_INIT_DURATION, 1,
                                        (rtimer_callback_t)chaos_scheduler, ptr);
                    } else {
                        // The reference time was updated:
                        // Schedule begin of next Chaos phase based on reference time and CHAOS_INIT_PERIOD.
                        // MM: (Period-20) ms
                        rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD - CHAOS_INIT_GUARD_TIME,
                                        (rtimer_callback_t)chaos_scheduler, ptr);
                    }
                } else {
                    // Chaos has already successfully bootstrapped:
                    // Schedule begin of next Chaos phase based on reference time and CHAOS_PERIOD.
                    // MM: Period-(1+syncmissed) ms
                    rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD +
                                    period_skew - CHAOS_GUARD_TIME * (1 + sync_missed),
                                    (rtimer_callback_t)chaos_scheduler, ptr);
                }
                // Yield the protothread.
                PT_YIELD(&pt);

                // // fixed time at the end
                rtimer_set_long(t, RTIMER_TIME(t) + FIXED_RADIO_OFF_DURATION, 1,
                (rtimer_callback_t)chaos_scheduler, ptr);
                PT_YIELD(&pt);
            } else{
                // chaos duty cycling mode
                while(1){
                    // sampling duty cycling
                    while(1){
                        // change channel every two rounds
                        #if CHANNEL_HOPPING
                        if(switched % 2 == 0){
                            cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                            curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                        }
                        switched++;
                        #endif

                        cc2420_radio_on();

                        // sleep for 0.2 ms
                        rtimer_set(t, RTIMER_TIME(t) + SAMPLING_RADIO_ON_DURATION, 1,
                        (rtimer_callback_t)chaos_scheduler, ptr);
                        PT_YIELD(&pt);

                        cca = cc2420_cca();

                        cc2420_radio_off();

                        // printf("%d\n", cca);
                        // cca_log[cca_index] = cca;
                        // cca_index = (cca_index + 1) % 100;

                        if(!cca)
                            break;

                        if(cca_switched % NUM_OF_SAMPLING == 0){
                        	// sleep for 100 ms
                            rtimer_set(t, RTIMER_TIME(t) + SAMPLING_RADIO_LONG_OFF_DURATION, 1,
                            (rtimer_callback_t)chaos_scheduler, ptr);
                            PT_YIELD(&pt);
                        } else{
                        	// sleep for 1 ms
                            rtimer_set(t, RTIMER_TIME(t) + SAMPLING_RADIO_OFF_DURATION, 1,
                            (rtimer_callback_t)chaos_scheduler, ptr);
                            PT_YIELD(&pt);
                        }
                        cca_switched++;
                    }

                    // change channel every two rounds
                    #if CHANNEL_HOPPING
                        if(switched % 2 == 0){
                            cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                            curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                        }
                        switched++;
                    #endif

                    channel_is_free = 1;
                    setArrayIndex();
                    ofge_init(node_index);
                    just_started = JUST_STARTED;

                    chaos_start((uint8_t *)&chaos_data, /*DATA_LEN,*/ CHAOS_RECEIVER, /*CHAOS_SYNC,*/ N_TX);
                    t_start = RTIMER_TIME(t);

                    // wait for 50ms and check if something is received
                    rtimer_set(t, RTIMER_TIME(t) + RADIO_ON_DURATION, 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    PT_YIELD(&pt);

                    // if something is received, break
                    // else, stop_chaos now, wait for 100ms and start again
                    if(!channel_is_free){
                        break;
                    } else{
                        chaos_stop();
                        /*rtimer_set(t, RTIMER_TIME(t) + RADIO_OFF_DURATION, 1,
                                (rtimer_callback_t)chaos_scheduler, ptr);
                        PT_YIELD(&pt);*/
                    }
                }

                if (CHAOS_IS_BOOTSTRAPPING()) {
                    // Chaos is still bootstrapping:
                    // Schedule end of Chaos phase based on CHAOS_INIT_DURATION.
                    // MM: Duration+49ms
                    #if CHANNEL_HOPPING
                    for(i = 0; i < NUM_OF_100_DURATION; i++){
                        channel_is_free = 1;
                        rtimer_set(t,  RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                        // Yield the protothread.
                        PT_YIELD(&pt);

                        if(channel_is_free){
                                cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                                curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                        }
                    }
                    rtimer_set(t, RTIMER_TIME(t) + (CHAOS_INIT_GUARD_TIME - CHAOS_GUARD_TIME), 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    #else
                    rtimer_set(t, RTIMER_TIME(t) + CHAOS_INIT_DURATION, 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    #endif
                } else {
                    // Chaos has already successfully bootstrapped:
                    // Schedule end of Chaos phase based on CHAOS_DURATION.
                    // MM: Duration+(1+syncmissed) ms
                    #if CHANNEL_HOPPING
                    for(i = 0; i < NUM_OF_100_DURATION; i++){
                        channel_is_free = 1;
                        rtimer_set(t,  RTIMER_TIME(t) + SRC_RETRANSMISSION_ON_DURATION, 1, (rtimer_callback_t)chaos_scheduler, ptr);
                        // Yield the protothread.
                        PT_YIELD(&pt);

                        if(channel_is_free){
                            cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
                            curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
                        }
                    }
                    rtimer_set(t, RTIMER_TIME(t) + + CHAOS_GUARD_TIME * (1 + sync_missed), 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    #else
                    rtimer_set(t, RTIMER_TIME(t) + CHAOS_GUARD_TIME * (1 + sync_missed) + CHAOS_DURATION, 1,
                    (rtimer_callback_t)chaos_scheduler, ptr);
                    #endif
                }
                // Yield the protothread.
                PT_YIELD(&pt);

                chaos_stop();

                if (CHAOS_IS_BOOTSTRAPPING()) {
                    // Chaos is still bootstrapping.
                    if (!CHAOS_IS_SYNCED()) {
                        // The reference time was not updated: reset skew_estimated to zero.
                        skew_estimated = 0;
                    }
                } else {
                    // Chaos has already successfully bootstrapped.
                    if (!CHAOS_IS_SYNCED()) {
                        // The reference time was not updated:
                        // increment reference time by CHAOS_PERIOD + period_skew.
                        set_t_ref_l(CHAOS_REFERENCE_TIME + CHAOS_PERIOD + period_skew);
                        set_t_ref_l_updated(1);
                        // Increment sync_missed.
                        sync_missed++;
                    } else {
                        // The reference time was not updated: reset sync_missed to zero.
                        sync_missed = 0;
                    }

                    // Poll the process that prints statistics (will be activated later by Contiki).
                    process_poll(&chaos_print_stats_process);
                }


                // update periodic_mode
                // if((num_no_update + num_solved) == CHAOS_NODES)
                //         periodic_mode = 0;
                // else
                //         periodic_mode = 1;

                // Estimate the clock skew over the last period.
                estimate_period_skew();
                
                // after chaos_stop, wait for sometime to make sure all nodes chaos_stop
                
                //*** help sync it ***//
                // if still bootstapping, I should wait more time
                // depending on bootstrapping and skew
                if (CHAOS_IS_BOOTSTRAPPING()) {
                    // Chaos is still bootstrapping.
                    if (skew_estimated == 0) {
                        // The reference time was not updated:
                        // Schedule begin of next Chaos phase based on last begin and CHAOS_INIT_PERIOD.
                        // MM: 10 ms
                        rtimer_set(t, RTIMER_TIME(t) + CHAOS_INIT_PERIOD - CHAOS_INIT_DURATION, 1,
                                        (rtimer_callback_t)chaos_scheduler, ptr);
                    } else {
                        // The reference time was updated:
                        // Schedule begin of next Chaos phase based on reference time and CHAOS_INIT_PERIOD.
                        // MM: (Period-20) ms
                        rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD - CHAOS_INIT_GUARD_TIME,
                                        (rtimer_callback_t)chaos_scheduler, ptr);
                    }
                } else {
                    // Chaos has already successfully bootstrapped:
                    // Schedule begin of next Chaos phase based on reference time and CHAOS_PERIOD.
                    // MM: Period-(1+syncmissed) ms
                    rtimer_set_long(t, CHAOS_REFERENCE_TIME, CHAOS_PERIOD +
                                    period_skew - CHAOS_GUARD_TIME * (1 + sync_missed),
                                    (rtimer_callback_t)chaos_scheduler, ptr);
                }
                // Yield the protothread.
                PT_YIELD(&pt);

                // // fixed time at the end
                rtimer_set_long(t, RTIMER_TIME(t) + FIXED_RADIO_OFF_DURATION, 1,
                (rtimer_callback_t)chaos_scheduler, ptr);
                PT_YIELD(&pt);
            }
		}
	}
	PT_END(&pt);
}

/** @} */

/**
 * \defgroup chaos-test-init Initialization
 * @{
 */

static uint8_t init_mapping(uint16_t nodeID);

#ifdef NODE_ID_MAPPING

//mapping of node id to flag
static const uint16_t mapping[] = NODE_ID_MAPPING;

//execute mapping of node id to flag
static uint8_t init_mapping(uint16_t nodeID){
	unsigned int i;
	//lookup id
	#if TOTAL_NODES_RUN
		for(i = 0; i < TOTAL_NODES; i++ ){
	#else
		for(i = 0; i < CHAOS_NODES; i++){
	#endif
		if(nodeID == mapping[i]){
			node_index = i;
			has_data = 1;
			return 1;
		}
	}
	has_data = 0;
	return 1;
}

#else

//execute mapping of node id to flag
static uint8_t init_mapping(uint16_t nodeID){
	//simple flags: node id minus one
	node_index = nodeID - 1;
	#if TOTAL_NODES_RUN
		if(node_index < TOTAL_NODES) {
	#else
		if(node_index<CHAOS_NODES) {
	#endif
		has_data = 1;
	} else {
		has_data = 0;
	}
	return 1;
}

#endif

// ebramkw
// collect data random
// static void collect_data_random() {
// 	int i, j;
// 	for(i = 0; i < M; i++) {
// 		for(j = 0; j < N; j++) {
// 			total_approximated_data_matrix[node_index][i][j] = (float)random_rand() / 1000.0;
// 		}
// 	}
// }

void local_svd() {
	// ebramkw
	// calculate local svd
	static int i, j;
	
	//compute mean vector
	float mean_vector[N]; //mean vector
	for(i = 0; i < N; i++)
		mean_vector[i] = 0;
	for(j = 0; j < N; j++) {
		for(i = 0; i < M; i++){
			mean_vector[j] += data_matrix[i][j];
		}
		mean_vector[j] /= M;
	}

	//data matrix with zero mean
	for(j = 0; j < N; j++)
		for(i = 0; i < M; i++)
			data_matrix[i][j] -= mean_vector[j];

	// free(mean_vector);

	//local svd
	float singular_values[N];
	float singular_vectors[N][N];
	i = dsvd(data_matrix, M, N, singular_values, singular_vectors);

	singular_value = singular_values[0];
	for(i = 0; i < N; i++)
		singular_vector[i] = singular_vectors[i][0];

	// printing
    static int dec;
	static float frac;
	// print eigenvalues (decreasing order)
	printf("local eigenvalues: ", node_id);
	for(i = 0; i < N; i++) {
		dec = singular_values[i];
		frac = singular_values[i] - dec;
		printf("%d.%04u ", dec, (unsigned int)(frac * 10000));
	}
	printf("\n");

	// print corresponding eigenvectors (each vector is in a row)
	for(i = 0; i < N; i++) {
		printf("local eigenvector %d: ", i);
		for(j = 0; j < N; j++) {
			dec = singular_vectors[i][j];
			frac = singular_vectors[i][j] - dec;
			printf("%d.%04u ", dec, (unsigned int)(frac * 10000));
		}
		printf("\n");
	}

	// if(result == 1) {
	// 	// printf("node %d computed the SVD\n", node_id);
	// 	Sort_by_Decreasing_Singular_Values(N, singular_values, singular_vectors);
	// } else {
	// 	printf("Node %d has an error computing SVD\n", node_id);
	// }
}

// void project_own_data(){
// 	// project data matrix on global pca
// 	int i, c, k;

// 	memset(total_projected_data_matrix, 0, CHAOS_NODES * M * sizeof(float));

// 	// for(i = 0; i < CHAOS_NODES; i++){
// 	// 	for (c = 0; c < M; c++) {
// 	// 		total_projected_data_matrix[i][c][0] = 0;
// 	// 	}
// 	// }

// 	if(has_data) {
// 		for (c = 0; c < M; c++) {
// 			for (k = 0; k < N; k++) {
// 				total_projected_data_matrix[node_index][c][0] = total_projected_data_matrix[node_index][c][0] + total_approximated_data_matrix[node_index][c][k] * global_singular_vector[k];
// 			}
// 		}
// 	}
// }

// void get_approximated_data(){
// 	// will do it for all, all results in one matrix
// 	// *** don't edit my own matrix *** //
// 	int i;
// 	int c, d;
// 	for(i = 0; i < CHAOS_NODES; i++){
// 		if(i != node_index){
// 			for (c = 0; c < M; c++) {
// 				for (d = 0; d < N; d++) {
// 					total_approximated_data_matrix[i][c][d] = total_projected_data_matrix[i][c][0] * global_singular_vector[d];
// 				}
// 			}
// 		}
// 	}
// }

PROCESS(chaos_test, "Chaos test");
AUTOSTART_PROCESSES(&chaos_test);
PROCESS_THREAD(chaos_test, ev, data)
{
	PROCESS_BEGIN();

	// ebramkw
	init_mapping(node_id);
	
        // after calling, nodes has their index and has data flag

	// leds_off(LEDS_BLUE);
	
	// printf("PKT_LEN %d DATA_LEN %d\n", PACKET_LEN, DATA_LEN);

#ifdef TESTBED
	// wait a few seconds, so we can hopefully read the initial message
	static struct etimer et;
    etimer_set(&et, 10 * CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
#endif /* TESTBED */

    if (IS_INITIATOR()) {
    	// leds_on(LEDS_RED);
    	static struct etimer et;
#ifdef TESTBED
		etimer_set(&et, 120 * CLOCK_SECOND);
#else
		etimer_set(&et, 1 * CLOCK_SECOND);
#endif		
		PROCESS_WAIT_UNTIL(etimer_expired(&et));
		// leds_off(LEDS_RED);
    }

	printf("chaos test! tx power: %u, proc cycles: %u, timeouts: %u, max timeout: %u, min timeout: %u, tx on complete: %d, payload: %d, MERGE_LEN %d, MERGE %d, period %lu, duration %lu, node count: %u\n", CC2420_TXPOWER, (uint16_t)PROCESSING_CYCLES, TIMEOUT, MAX_SLOTS_TIMEOUT, MIN_SLOTS_TIMEOUT, N_TX_COMPLETE, PAYLOAD_LEN, MERGE_LEN, MERGE, CHAOS_PERIOD, CHAOS_DURATION, CHAOS_NODES);
	printf("node_id: %u, node_index %u\n", node_id, node_index);
	
	// Initialize Chaos data.
	chaos_data.seq_no = 0;

	CHAOS_BOOTSTRAPPING = 1;

    has_data_update = 0;

    periodic_mode = 0;
    curr_periodic_rounds = 0;

	curr_payload_len = ((SHARE_SV * N) + SHARE_SV) * 4;

    channel_is_free = 1;
    src_got_answer = 0;

    source_node_index = 0;

    cca_switched = - NUM_OF_SAMPLING;

    #if CHANNEL_HOPPING
        curr_rf_channel = 0;
        switched = -2;
    #endif

    #if CHAOS_SETUP
        curr_setup_rounds = 0;
    #endif

    send_data_now = 0;

    // cca_index = 0;

	// ofge_init(node_index);
	// Start print stats processes.
	process_start(&chaos_print_stats_process, NULL);
	// Start Chaos busy-waiting process.
	process_start(&chaos_process, NULL);

    if(has_data){
    	#if !ONE_SRC
    		has_data_update = 1;
    	#endif
    	process_start(&sensing_process, NULL);
    }

	// Start Chaos experiment in one second.
	rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND, 1, (rtimer_callback_t)chaos_scheduler, NULL);

	PROCESS_END();
}

/** @} */
/** @} */
/** @} */

// uint8_t my_var = 240;						//11110000:240
// printf("my_var: %u\n",my_var);				//11110000:240
// uint8_t var = 170;							//10101010:170
// printf("old_var: %u\n",var);					//10101010:170
// uint8_t new_var = ~var;						//01010101: 85
// printf("inv_var: %u\n",new_var);				//01010101: 85
// new_var = new_var&my_var;					//01010000: 80
// printf("new_var: %u\n",new_var);				//01010000: 80

// while(1) {
// 	printf("%d\n",(((rand()+32768) % 10)==0));
// }