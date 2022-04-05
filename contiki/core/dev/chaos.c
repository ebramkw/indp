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
 * \file
 *         Chaos core, source file.
 * \author
 *         Olaf Landsiedel <olafl@chalmers.se>
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#include "chaos.h"
#include "indp-test.h"
#include "oofg.h"

/**
 * \brief a bunch of define for gcc 4.6
 */
#define CM_POS              CM_1
#define CM_NEG              CM_2
#define CM_BOTH             CM_3

static uint8_t initiator, /*sync,*/ rx_cnt, tx_cnt, tx_max;
static uint8_t *data, *packet;
//static uint8_t data_len, packet_len;
static uint8_t bytes_read, tx_relay_cnt_last;
static volatile uint8_t state;
static rtimer_clock_t t_rx_start, t_rx_stop, t_tx_start, t_tx_stop;
static rtimer_clock_t t_rx_timeout;
static rtimer_clock_t T_irq;
static unsigned short ie1, ie2, p1ie, p2ie, tbiv;
static uint8_t tx;
static uint8_t chaos_complete;
static uint8_t tx_cnt_complete;
static uint8_t estimate_length;
static rtimer_clock_t t_timeout_start, t_timeout_stop, now, tbccr1;
static uint32_t T_timeout_h;
static uint16_t n_timeout_wait;
static uint8_t n_slots_timeout, relay_cnt_timeout;

static rtimer_clock_t T_slot_h = 0, T_rx_h, T_w_rt_h, T_tx_h, T_w_tr_h, t_ref_l, T_offset_h, t_first_rx_l;
#if CHAOS_SYNC_WINDOW
static unsigned long T_slot_h_sum;
static uint8_t win_cnt;
#endif /* CHAOS_SYNC_WINDOW */
static uint8_t relay_cnt, t_ref_l_updated;

#ifdef LOG_TIRQ
#define CHAOS_TIRQ_LOG_SIZE 70
static uint8_t tirq_log_cnt;
static rtimer_clock_t tirq_log[CHAOS_TIRQ_LOG_SIZE];
#endif //LOG_TIRQ

#ifdef LOG_FLAGS
#define CHAOS_FLAGS_LOG_SIZE 70
static uint16_t flags_tx_cnt;
static uint16_t flags_rx_cnt;
static uint8_t flags_tx[CHAOS_FLAGS_LOG_SIZE*MERGE_LEN];
static uint8_t relay_counts_tx[CHAOS_FLAGS_LOG_SIZE];
static uint8_t flags_rx[CHAOS_FLAGS_LOG_SIZE*MERGE_LEN];
static uint8_t relay_counts_rx[CHAOS_FLAGS_LOG_SIZE];
#ifdef LOG_ALL_FLAGS
static uint8_t current_flags_rx[MERGE_LEN];
#endif /* LOG_ALL_FLAGS */
#endif /* LOG_FLAGS */

#define CHAOS_SOLVED_LOG_SIZE 250
static uint16_t solved_cnt = 0;
static int solve_progress[CHAOS_SOLVED_LOG_SIZE];
static int solve_source[CHAOS_SOLVED_LOG_SIZE];
static int dclog[CHAOS_SOLVED_LOG_SIZE];
static int degreelog[CHAOS_SOLVED_LOG_SIZE];

// M: Parameters to prevent tx close to end of chaos phase
//#define ELAPSED 47500
//#define BUFFER  50
#define ELAPSED 45500
#define BUFFER  150	//ms
rtimer_clock_t buffer = RTIMER_SECOND/BUFFER; //ticks
rtimer_clock_t t_now;
rtimer_clock_t t_begin;
rtimer_clock_t elapsed;

// M: Packet selection
chaos_data_struct* received;

// M: TX Power
int power_levels[] = {24, 31};

static inline void chaos_schedule_timeout(void) {
	// M: Schedule the timeout only if T_slot_h is known
	if (T_slot_h && TIMEOUT) {
		// M: random number between MIN_SLOTS_TIMEOUT and MAX_SLOTS_TIMEOUT
		// M: RTIMER_NOW() + RTIMER_NOW_DCO() is used to randomize perhaps
		n_slots_timeout = MIN_SLOTS_TIMEOUT + ((RTIMER_NOW() + RTIMER_NOW_DCO()) % (MAX_SLOTS_TIMEOUT - MIN_SLOTS_TIMEOUT + 1));
		T_timeout_h = n_slots_timeout * (uint32_t)T_slot_h;
		t_timeout_stop = t_timeout_start + T_timeout_h;
		// M: Is this the code to wrap around in case the wait is longer than what timer can capture
		// M: Indicates how many timer_max one needs to wait => n_timeout_wait
		if (T_timeout_h >> 16) {
			now = RTIMER_NOW_DCO();
			n_timeout_wait = (T_timeout_h - (now - t_timeout_start)) >> 16;
	        // it should never happen, but to be sure...
			if (n_timeout_wait == 0xffff) {
				n_timeout_wait = 0;
			}
		} else {
			n_timeout_wait = 0;
		}
		// M: It starts a periodic timer which keeps firing until disabled
		TBCCR4 = t_timeout_stop;
		TBCCTL4 = CCIE;
		SET_PIN_ADC6;
	}
}

static inline void chaos_stop_timeout(void) {
	TBCCTL4 = 0;
	n_timeout_wait = 0;
	UNSET_PIN_ADC6;
}

/* --------------------------- Radio functions ---------------------- */

static inline void radio_flush_tx(void) {
	FASTSPI_STROBE(CC2420_SFLUSHTX);
}

static inline uint8_t radio_status(void) {
	uint8_t status;
	FASTSPI_UPD_STATUS(status);
	return status;
}

static inline void radio_on(void) {
	FASTSPI_STROBE(CC2420_SRXON);
	while(!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));
	SET_PIN_ADC0;
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
}

static inline void radio_off(void) {
#if ENERGEST_CONF_ON
	if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
		ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
	}
	if (energest_current_mode[ENERGEST_TYPE_LISTEN]) {
		ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	}
#endif /* ENERGEST_CONF_ON */
	UNSET_PIN_ADC0;
	UNSET_PIN_ADC1;
	UNSET_PIN_ADC2;
	FASTSPI_STROBE(CC2420_SRFOFF);
	chaos_stop_timeout();
}

static inline void radio_flush_rx(void) {
	uint8_t dummy;
	FASTSPI_READ_FIFO_BYTE(dummy);
	FASTSPI_STROBE(CC2420_SFLUSHRX);
	FASTSPI_STROBE(CC2420_SFLUSHRX);
}

static inline void radio_abort_rx(void) {
	state = CHAOS_STATE_ABORTED;
	UNSET_PIN_ADC1;
	radio_flush_rx();
}

static inline void radio_abort_tx(void) {
	UNSET_PIN_ADC2;
	FASTSPI_STROBE(CC2420_SRXON);
#if ENERGEST_CONF_ON
	if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
		ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
		ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	}
#endif /* ENERGEST_CONF_ON */
	radio_flush_rx();
}

static inline void radio_start_tx(void) {
	t_timeout_start = RTIMER_NOW_DCO();

	// Mobashir: stop tx close to the end of chaos phase
	t_now = RTIMER_NOW();
	if(t_begin==0)
		t_begin = t_now;
	if((int)t_now > (int)t_begin) {
		elapsed = (t_now - t_begin);		
	} else {
		elapsed = (65536 - t_begin) + t_now;
	}
	if(ELAPSED <= (elapsed + buffer)) {
		// leds_on(LEDS_RED);
		return;
	}

	FASTSPI_STROBE(CC2420_STXON);
	SET_PIN_ADC0;							//M: Radio ON
	UNSET_PIN_ADC1;							//M: RX OFF? 
#if ENERGEST_CONF_ON
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif /* ENERGEST_CONF_ON */
	chaos_schedule_timeout();
}

static inline void radio_write_tx(void) {
	FASTSPI_WRITE_FIFO(packet, PACKET_LEN - 1);
}

void chaos_data_processing(void) {

	rtimer_clock_t start_time = tbccr1;

	received = (chaos_data_struct*)(&CHAOS_DATA_FIELD);

	// decide if I need to wake up and participate in this round
	if(channel_is_free){
		if(received->seq_no > curr_seq){
			channel_is_free = 0;
		} else{
			tx = 0;
			return;
		}
	}

	// // Get current relay
	// curr_relay = CHAOS_RELAY_CNT_FIELD;

	// Record how much nbrs solved so far
	// nbr_solved = received->solved;

	// compute it from no_update flag to save size
	// #if R_PCA
		// Record how much nbrs updated so far
		// nbr_updated = received->updated;
	// #endif

	// Record who transmitted
	nbr_source = received->source;

	// Record who is destination
	nbr_destination = received->destination;

	// Copy the solved bv
	memcpy(solved_bv, received->solved_bv, MERGE_LEN * sizeof(uint8_t));

	#if TOTAL_NODES_RUN
		memcpy(update_control_bv, received->no_update, TOTAL_MERGE_LEN * sizeof(uint8_t));
		memcpy(data_control_bv, received->data_control, TOTAL_MERGE_LEN * sizeof(uint8_t));
	#else
		memcpy(update_control_bv, received->no_update, MERGE_LEN * sizeof(uint8_t));
		memcpy(data_control_bv, received->data_control, MERGE_LEN * sizeof(uint8_t));
	#endif

	// ebramkw: used for debugging
	// memcpy(bv, received->bv, MERGE_LEN * sizeof(uint8_t));
	// memcpy(payload_v, received->payload, PAYLOAD_LEN * sizeof(uint8_t));

	// Decode and set my flags if I have solved
	#if R_PCA
		#if LOSSY_RPCA
			curr_elapsed = elapsed;
			if(decode(received->bv, received->solved_bv, received->payload, received->no_update, received->data_control)) {
				if(!only_once){
					// leds_on(LEDS_GREEN);
					// leds_on(LEDS_BLUE);
					// leds_on(LEDS_RED);
					actual_num_no_update = num_no_update;
					actual_num_solved = num_data;
					only_once = 1;
					// leds_off(LEDS_GREEN);
					// leds_off(LEDS_BLUE);
					// leds_off(LEDS_RED);
				}
		#else
			if(decode(received->bv, received->solved_bv, received->payload, received->no_update)) {
		#endif
	#else
		if(decode(received->bv, received->solved_bv, received->payload)) {
	#endif
		// leds_on(LEDS_BLUE);
		im_complete = 1;
		if(slots==0 && ticks==0) {
			slots = curr_slot;
			ticks = elapsed;
		}
	}

	// rtimer_clock_t mid_time = RTIMER_NOW_DCO();
	// decode_time = (mid_time - start_time)%65536;
	// if(dp_count<ACTIVITY_LOG_SIZE)
	// 	decode_time_log[dp_count] = decode_time;

	// Adding the stats
	if(solved_cnt < CHAOS_SOLVED_LOG_SIZE) {
		solve_progress[solved_cnt] = num_solved;
		solve_source[solved_cnt] = received->source;
		dclog[solved_cnt] = decode_count;
	}

	// Add to the packet how much I solved so far
	// received->solved = num_solved;

	// compute it from no_update flag to save size
	// Add to the packet how much I updated so far
	// #if R_PCA
		// received->updated = num_no_update;
	// #endif

	// Add to the packet the source node id
	received->source = node_id;
	
	// Record the current seq no
	curr_seq = received->seq_no;

	// Generate a single encoded pkt
	encode(received->bv, received->payload);

	// Check flags to see if all have solved and set chaos_complete
	uint16_t i;
	for( i = 0; i < MERGE_LEN-1; i++){
		//M: Added later to see cumulative results
#if defined LOG_FLAGS && defined LOG_ALL_FLAGS
		// current_flags_rx[i] = received->flags[i];
		current_flags_rx[i] = received->bv[i];
#endif /* LOG_FLAGS */	
		}
#if defined LOG_FLAGS && defined LOG_ALL_FLAGS
		// current_flags_rx[MERGE_LEN-1] = received->flags[MERGE_LEN-1];
		current_flags_rx[MERGE_LEN-1] = received->bv[MERGE_LEN-1];
#endif /* LOG_FLAGS */

	// Add intended destination
	received->destination = nbr_destination;

	// Copy the solved bv
	memcpy(received->solved_bv, solved_bv, MERGE_LEN * sizeof(uint8_t));

	#if TOTAL_NODES_RUN
		memcpy(received->no_update, update_control_bv, TOTAL_MERGE_LEN * sizeof(uint8_t));
		memcpy(received->data_control, data_control_bv, TOTAL_MERGE_LEN * sizeof(uint8_t));
	#else
		memcpy(received->no_update, update_control_bv, MERGE_LEN * sizeof(uint8_t));
		memcpy(received->data_control, data_control_bv, MERGE_LEN * sizeof(uint8_t));
	#endif

	// Adding further stats
	if(solved_cnt < CHAOS_SOLVED_LOG_SIZE) {
		degreelog[solved_cnt] = curr_degree;
	}
	solved_cnt++;

	// Mobashir: stop tx close to the end of chaos phase
	t_now = RTIMER_NOW();
	if(t_begin==0)
		t_begin = t_now;
	if((int)t_now > (int)t_begin) {
		elapsed = (t_now - t_begin);
	} else {
		elapsed = (65536 - t_begin) + t_now;
	}

	int time_exceeded  = (ELAPSED <= (elapsed + buffer));

	if(time_exceeded) {
		tx = 0;
	} else {
		if(just_started) {
        	tx = 1;
        	just_started--;
        } else {		
			// ebramkw
			// increasing what's after the %, gives less zeros
			// #if LOSSY_RPCA
			// 	if(half_decoded_new == 1){
			// 		tx_cond[0][0][0] = (((rand()+32768) % 2)==0);
			// 		tx_cond[1][0][0] = (((rand()+32768) % 2)==0);
			// 	} else if (half_decoded_new != 0){
			// 		tx_cond[0][0][0] = (((rand()+32768) % 3)==0);
			// 		tx_cond[1][0][0] = (((rand()+32768) % 3)==0);
			// 	} else {
			// 		// give highr priority for new update than nothing
			// 		if(update_decoded_new && num_solved > 0){
			// 			// new update, higher probability to send
			// 			tx_cond[0][0][0] = (((rand()+32768) % 4)==0);
			// 			tx_cond[1][0][0] = (((rand()+32768) % 4)==0);
			// 		} else{
			// 			// nothing, less probability to send
			// 			tx_cond[0][0][0] = (((rand()+32768) % 13)==0);
			// 			tx_cond[1][0][0] = (((rand()+32768) % 13)==0);
			// 		}
			// 	}
			// #else
				tx_cond[0][0][0] = (((rand()+32768) % 10)==0);
				tx_cond[1][0][0] = (((rand()+32768) % 10)==0);
			// #endif
			// ebramkw, should increase 3 to give less zeros so the result (tx) is 0
			// ebramkw, termination condition but keep sending not to loos sync
			tx_cond[1][1][0] = (((rand()+32768) % 3)==0); //0 Ideally

			tx = tx_cond[im_complete][neighbor_complete][decoded_new];
		}
	}

	// #if LOSSY_R_PCA
	// 	if(num_not_decoded == MAX_NUMBER_OF_NOT_DECODED_NEW)
	// 		tx = 0;
	// #endif

	//if(tx) {
	//	cc2420_set_tx_power(power_levels[(((rand()+32768) % 2)==0)]);
	//}
	
	// if((time_exceeded)||(neighbor_complete&&im_complete))
	// 	leds_on(LEDS_RED);
	// else
	// 	leds_off(LEDS_RED);

	// rtimer_clock_t stop_time = RTIMER_NOW_DCO();
	// encode_time = (stop_time - mid_time)%65536;
	// if(dp_count<ACTIVITY_LOG_SIZE)
	// 	encode_time_log[dp_count++] = encode_time;
}

/* --------------------------- SFD interrupt ------------------------ */
interrupt(TIMERB1_VECTOR) __attribute__ ((section(".chaos")))
timerb1_interrupt(void)
{

	if (state == CHAOS_STATE_RECEIVING && !SFD_IS_1) {
		// MRecv2
		// packet reception has finished

		// store the time at which the SFD was captured into an ad hoc variable
		// (the value of TBCCR1 might change in case other SFD interrupts occur)
		tbccr1 = TBCCR1;

		UNSET_PIN_ADC1;

		// read the remaining bytes from the RXFIFO
		FASTSPI_READ_FIFO_NO_WAIT(&packet[bytes_read], PACKET_LEN - bytes_read + 1);
		bytes_read = PACKET_LEN + 1;

		if (CHAOS_CRC_FIELD & FOOTER1_CRC_OK) {
			// CRC ok: packet successfully received

			src_got_answer = 1;
			// channel_is_free = 0;

			#if CHANNEL_HOPPING
				// done receiving ==> change channel
				if(curr_rf_channel == 0)
					recevied_num_per_channel[2]++;
				else if(curr_rf_channel == 1)
					recevied_num_per_channel[0]++;
				else if(curr_rf_channel == 2)
					recevied_num_per_channel[1]++;

				cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
				curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
			#endif


			SET_PIN_ADC7;
			// stop the timeout
			chaos_stop_timeout();
			// data processing
			chaos_data_processing(); 

			//ok, data processing etc is done and we are ready to transmit a packet
			//now the black magic part starts:
			//we ensure synchronous transmissions on all transmitting Chaos nodes
			//by making them transmit the data packet at exactly the same number of
			//processor cycles after the receive
			//this consists of two steps:
			//1. we wait until a defined time is reached (wait loop)
			//2. we execute a NOP loop to ensure precise synchronization at CPU instruction cycles.
			//(step two is the same as in Glossy)

			// MM: Timer B sourced by the DCO and uses UP mode
			// MM: Tuts -> http://www.crash-bang.com/getting-started-msp430-timers-3/
			// wait loop
			// wait until PROCESSING_CYCLES cycles occurred since the last SFD event

			TBCCR4 = tbccr1 + PROCESSING_CYCLES;
			do {
				TBCCTL4 |= CCIE;
			} while (!(TBCCTL4 & CCIFG));
			TBCCTL4 = 0;

			UNSET_PIN_ADC7;

			// the next instruction is executed at least PROCESSING_CYCLES+12 cycles since the last SFD event
			// ->achieve basic clock synchronization for synchronous TX

			//prepare for NOP loop
			//compute interrupt etc. delay to do get instruction level synchronization for TX
			T_irq = ((RTIMER_NOW_DCO() - tbccr1) - (PROCESSING_CYCLES+15)) << 1;

			// NOP loop: slip stream!!
			// if delay is within reasonable range: execute NOP loop do ensure synchronous TX
			// T_irq in [0,...,34]
			if (T_irq <= 34) {
				if (tx) {
					// NOPs (variable number) to compensate for the interrupt service and the busy waiting delay
					asm volatile("add %[d], r0" : : [d] "m" (T_irq));
					asm volatile("nop");						// irq_delay = 0
					asm volatile("nop");						// irq_delay = 2
					asm volatile("nop");						// irq_delay = 4
					asm volatile("nop");						// irq_delay = 6
					asm volatile("nop");						// irq_delay = 8
					asm volatile("nop");						// irq_delay = 10
					asm volatile("nop");						// irq_delay = 12
					asm volatile("nop");						// irq_delay = 14
					asm volatile("nop");						// irq_delay = 16
					asm volatile("nop");						// irq_delay = 18
					asm volatile("nop");						// irq_delay = 20
					asm volatile("nop");						// irq_delay = 22
					asm volatile("nop");						// irq_delay = 24
					asm volatile("nop");						// irq_delay = 26
					asm volatile("nop");						// irq_delay = 28
					asm volatile("nop");						// irq_delay = 30
					asm volatile("nop");						// irq_delay = 32
					asm volatile("nop");						// irq_delay = 34
					// relay the packet
					//
					// -> all transmitting nodes have instruction level synchronization
					// with
					radio_start_tx();
				}
				// read TBIV to clear IFG
				tbiv = TBIV;
				chaos_end_rx();
			} else {
				// interrupt service delay is too high: do not relay the packet
				// FF: this should never happen!
				// leds_toggle(LEDS_RED);
#if CHAOS_DEBUG
				if (tx) {
					high_T_irq++;
				}
#endif
#ifdef LOG_TIRQ
				if (tirq_log_cnt < CHAOS_TIRQ_LOG_SIZE) {
					tirq_log[tirq_log_cnt] = T_irq;
					tirq_log_cnt++;
				}
#endif
				tx = 0;
				// read TBIV to clear IFG
				tbiv = TBIV;
				chaos_end_rx();
			}
		} else {
			// CRC not ok
#if CHAOS_DEBUG
			bad_crc++;
#endif /* CHAOS_DEBUG */
			tx = 0;
			// read TBIV to clear IFG
			tbiv = TBIV;
			chaos_end_rx();
		}
	} else {
		// read TBIV to clear IFG
		tbiv = TBIV;
		if (state == CHAOS_STATE_WAITING && SFD_IS_1) {
			// MRecv1
			// packet reception has started
			chaos_begin_rx();
		} else {
			if (state == CHAOS_STATE_RECEIVED && SFD_IS_1) {
				// MRecv3 and MTrans1
				// packet transmission has started
				chaos_begin_tx();
			} else {
				if (state == CHAOS_STATE_TRANSMITTING && !SFD_IS_1) {
					// MRecv4 and MTrans2
					// packet transmission has finished
					chaos_end_tx();

					#if CHANNEL_HOPPING
						cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
						curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
					#endif
				} else {
					if (state == CHAOS_STATE_ABORTED) {
						// packet reception has been aborted
						state = CHAOS_STATE_WAITING;

						#if CHANNEL_HOPPING
							cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
							curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
						#endif
					} else {
						// MRecv Timeout
						if (tbiv == TBIV_TBCCR4) {
							// timeout
							if (n_timeout_wait > 0) {
								n_timeout_wait--;
							} else {
								if (state == CHAOS_STATE_WAITING) {
									// start another transmission
									radio_start_tx();
									UNSET_PIN_ADC6;
									if (initiator && rx_cnt == 0) {
										CHAOS_LEN_FIELD = PACKET_LEN;
										CHAOS_HEADER_FIELD = CHAOS_HEADER;
									} else {
										// stop estimating the slot length during this round (to keep maximum precision)
										estimate_length = 0;
										CHAOS_LEN_FIELD = PACKET_LEN;
										CHAOS_HEADER_FIELD = CHAOS_HEADER+1;
									}
									CHAOS_RELAY_CNT_FIELD = relay_cnt_timeout;
									if (DATA_LEN > BYTES_TIMEOUT) {
										// first BYTES_TIMEOUT bytes
										memcpy(&CHAOS_DATA_FIELD, data, BYTES_TIMEOUT);
										radio_flush_rx();
										FASTSPI_WRITE_FIFO(packet, BYTES_TIMEOUT + 1 + CHAOS_HEADER_LEN);
										// remaining bytes
										memcpy(&CHAOS_BYTES_TIMEOUT_FIELD, &data[BYTES_TIMEOUT], DATA_LEN - BYTES_TIMEOUT);
										FASTSPI_WRITE_FIFO(&packet[BYTES_TIMEOUT + 1 + CHAOS_HEADER_LEN], PACKET_LEN - BYTES_TIMEOUT - 1 - CHAOS_HEADER_LEN - 1);
									} else {
										memcpy(&CHAOS_DATA_FIELD, data, DATA_LEN);
										// write the packet to the TXFIFO
										radio_flush_rx();
										radio_write_tx();
									}
									state = CHAOS_STATE_RECEIVED;
								} else {
									// stop the timeout
									chaos_stop_timeout();
								}
							}
						} else {
							if (state != CHAOS_STATE_OFF) {
								// something strange is going on: go back to the waiting state
								radio_flush_rx();
								// stop the timeout
								chaos_stop_timeout();
								state = CHAOS_STATE_WAITING;

								#if CHANNEL_HOPPING
									cc2420_set_channel(RF_CHANNELS[curr_rf_channel]);
									curr_rf_channel = (curr_rf_channel + 1) % CHANNELS_NUM;
								#endif
							}
						}
					}
				}
			}
		}
	}
}

/* --------------------------- Chaos process ----------------------- */
PROCESS(chaos_process, "Chaos busy-waiting process");
PROCESS_THREAD(chaos_process, ev, data) {
	PROCESS_BEGIN();

	// initialize output GPIO pins:
	// radio on
	INIT_PIN_ADC0_OUT;
	// packet Rx
	INIT_PIN_ADC1_OUT;
	// packet Tx
	INIT_PIN_ADC2_OUT;
	// Rx or Tx failure
	INIT_PIN_ADC6_OUT;
	// successful packet Rx
	INIT_PIN_ADC7_OUT;

	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
		// prevent the Contiki main cycle to enter the LPM mode or
		// any other process to run while Chaos is running
		while (CHAOS_IS_ON());
	}

	PROCESS_END();
}

static inline void chaos_disable_other_interrupts(void) {
    int s = splhigh();
	ie1 = IE1;
	ie2 = IE2;
	p1ie = P1IE;
	p2ie = P2IE;
	IE1 = 0;
	IE2 = 0;
	P1IE = 0;
	P2IE = 0;
	CACTL1 &= ~CAIE;
	DMA0CTL &= ~DMAIE;
	DMA1CTL &= ~DMAIE;
	DMA2CTL &= ~DMAIE;
	// disable etimer interrupts
	TACCTL1 &= ~CCIE;
	TBCCTL0 = 0;
	DISABLE_FIFOP_INT();
	CLEAR_FIFOP_INT();
	SFD_CAP_INIT(CM_BOTH);
	ENABLE_SFD_INT();
	// stop Timer B
	TBCTL = 0;
	// Timer B sourced by the DCO
	TBCTL = TBSSEL1;
	// start Timer B
	TBCTL |= MC1;
    splx(s);
    watchdog_stop();
}

static inline void chaos_enable_other_interrupts(void) {
	int s = splhigh();
	IE1 = ie1;
	IE2 = ie2;
	P1IE = p1ie;
	P2IE = p2ie;
	// enable etimer interrupts
	TACCTL1 |= CCIE;
#if COOJA
	if (TACCTL1 & CCIFG) {
		etimer_interrupt();
	}
#endif
	DISABLE_SFD_INT();
	CLEAR_SFD_INT();
	FIFOP_INT_INIT();
	ENABLE_FIFOP_INT();
	// stop Timer B
	TBCTL = 0;
	// Timer B sourced by the 32 kHz
	TBCTL = TBSSEL0;
	// start Timer B
	TBCTL |= MC1;
    splx(s);
    watchdog_start();
}

/* --------------------------- Main interface ----------------------- */
void chaos_start(uint8_t *data_, /*uint8_t data_len_,*/ uint8_t initiator_,
		/*uint8_t sync_,*/ uint8_t tx_max_) {
	// Mobashir: used to stop tx close to the end of chaos phase
	t_begin = 0;
	// copy function arguments to the respective Chaos variables
	data = data_;
//	data_len = data_len_;
	initiator = initiator_;
//	sync = sync_;
	tx_max = tx_max_;
	// disable all interrupts that may interfere with Chaos
	chaos_disable_other_interrupts();
	// initialize Chaos variables
	tx_cnt = 0;
	rx_cnt = 0;

	chaos_complete = CHAOS_INCOMPLETE;
	tx_cnt_complete = 0;
	estimate_length = 1;

#if CHAOS_DEBUG
	rc_update = 0;
#endif /* CHAOS_DEBUG */
	// set Chaos packet length, with or without relay counter depending on the sync flag value
	// packet_len = (CHAOS_SYNC_MODE) ?
	//			DATA_LEN + FOOTER_LEN + CHAOS_RELAY_CNT_LEN + CHAOS_HEADER_LEN :
	//			DATA_LEN + FOOTER_LEN + CHAOS_HEADER_LEN;

	// allocate memory for the temporary buffer
	packet = (uint8_t *) malloc(PACKET_LEN + 1);
	// set the packet length field to the appropriate value
	CHAOS_LEN_FIELD = PACKET_LEN;
	// set the header field
	CHAOS_HEADER_FIELD = CHAOS_HEADER;
	if (initiator) {
		// initiator: copy the application data to the data field
		// OL: from local data to packet that will be tx
		memcpy(&CHAOS_DATA_FIELD, data, DATA_LEN);
		// set Chaos state
		state = CHAOS_STATE_RECEIVED;
	} else {
		// receiver: set Chaos state
		state = CHAOS_STATE_WAITING;
	}
	if (CHAOS_SYNC_MODE) {
		// set the relay_cnt field to 0
		CHAOS_RELAY_CNT_FIELD = 0;
		// the reference time has not been updated yet
		t_ref_l_updated = 0;
	}

#if !COOJA
	// resynchronize the DCO
	msp430_sync_dco();
#endif /* COOJA */

	// flush radio buffers
	radio_flush_rx();
	radio_flush_tx();
	if (initiator) {
		// write the packet to the TXFIFO
		radio_write_tx();
		// start the first transmission
		radio_start_tx();
	} else {
		// turn on the radio
		radio_on();
	}
	// activate the Chaos busy waiting process
	process_poll(&chaos_process);
}

/*---------------------------------------------------------------------------*/
void cc2420_radio_on(void){
	radio_on();
}

void cc2420_radio_off(void){
	radio_off();
}

int cc2420_cca(void)
{
  int cca;

  // radio_on();

  while(!(BV(CC2420_RSSI_VALID)));

  cca = CCA_IS_1;

  // radio_off();
  
  return cca;
}

int cc2420_rssi(void)
{
  int rssi;

  // radio_on();

  while(!(BV(CC2420_RSSI_VALID)));

  rssi = (int)((signed char) cc2420_getreg(CC2420_RSSI));
  rssi += RSSI_OFFSET;

  // radio_off();
  
  return rssi;
}
/*---------------------------------------------------------------------------*/

uint8_t chaos_stop(void) {

	// turn off the radio
	radio_off();

	// flush radio buffers
	radio_flush_rx();
	radio_flush_tx();

	state = CHAOS_STATE_OFF;
	// re-enable non Chaos-related interrupts
	chaos_enable_other_interrupts();
	// deallocate memory for the temporary buffer
	free(packet);
	// return the number of times the packet has been received
	return rx_cnt;
}

uint8_t get_rx_cnt(void) {
	return rx_cnt;
}

uint8_t get_relay_cnt(void) {
	return relay_cnt;
}

rtimer_clock_t get_T_slot_h(void) {
	return T_slot_h;
}

uint8_t is_t_ref_l_updated(void) {
	return t_ref_l_updated;
}

rtimer_clock_t get_t_first_rx_l(void) {
	return t_first_rx_l;
}

rtimer_clock_t get_t_ref_l(void) {
	return t_ref_l;
}

void set_t_ref_l(rtimer_clock_t t) {
	t_ref_l = t;
}

void set_t_ref_l_updated(uint8_t updated) {
	t_ref_l_updated = updated;
}

uint8_t get_state(void) {
	return state;
}

static inline void estimate_slot_length(rtimer_clock_t t_rx_stop_tmp) {
	// estimate slot length if rx_cnt > 1
	// and we have received a packet immediately after our last transmission
	if ((rx_cnt > 1) && (CHAOS_RELAY_CNT_FIELD == (tx_relay_cnt_last + 2))) {
		T_w_rt_h = t_tx_start - t_rx_stop;
		T_tx_h = t_tx_stop - t_tx_start;
		T_w_tr_h = t_rx_start - t_tx_stop;
		T_rx_h = t_rx_stop_tmp - t_rx_start;
		uint32_t T_slot_h_tmp = ((uint32_t)T_tx_h + (uint32_t)T_w_tr_h + (uint32_t)T_rx_h + (uint32_t)T_w_rt_h) / 2;
#if CHAOS_SYNC_WINDOW
		T_slot_h_sum += T_slot_h_tmp;
		if ((++win_cnt) == CHAOS_SYNC_WINDOW) {
			// update the slot length estimation
			T_slot_h = T_slot_h_sum / CHAOS_SYNC_WINDOW;
			// halve the counters
			T_slot_h_sum /= 2;
			win_cnt /= 2;
		} else {
			if (win_cnt == 1) {
				// at the beginning, use the first estimation of the slot length
				T_slot_h = T_slot_h_tmp;
			}
		}
		estimate_length = 0;
#if CHAOS_DEBUG
		rc_update = CHAOS_RELAY_CNT_FIELD;
#endif /* CHAOS_DEBUG */
#else
		T_slot_h = T_slot_h_tmp;
#endif /* CHAOS_SYNC_WINDOW */
	}
}

static inline void compute_sync_reference_time(void) {
#if COOJA
	rtimer_clock_t t_cap_l = RTIMER_NOW();
	rtimer_clock_t t_cap_h = RTIMER_NOW_DCO();
#else
	// capture the next low-frequency clock tick
	rtimer_clock_t t_cap_h, t_cap_l;
	CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l);
#endif /* COOJA */
	rtimer_clock_t T_rx_to_cap_h = t_cap_h - t_rx_start;
	unsigned long T_ref_to_rx_h = (CHAOS_RELAY_CNT_FIELD - 1) * (unsigned long)T_slot_h;
	unsigned long T_ref_to_cap_h = T_ref_to_rx_h + (unsigned long)T_rx_to_cap_h;
	rtimer_clock_t T_ref_to_cap_l = 1 + T_ref_to_cap_h / CLOCK_PHI;
	// high-resolution offset of the reference time
	T_offset_h = (CLOCK_PHI - 1) - (T_ref_to_cap_h % CLOCK_PHI);
	// low-resolution value of the reference time
	t_ref_l = t_cap_l - T_ref_to_cap_l;
	relay_cnt = CHAOS_RELAY_CNT_FIELD - 1;
	// the reference time has been updated
	t_ref_l_updated = 1;
}

/* ----------------------- Interrupt functions ---------------------- */
inline void chaos_begin_rx(void) {
	SET_PIN_ADC1;
	t_rx_start = TBCCR1;
	state = CHAOS_STATE_RECEIVING;
	// Rx timeout: packet duration + 200 us
	// (packet duration: 32 us * packet_length, 1 DCO tick ~ 0.23 us)
	t_rx_timeout = t_rx_start + ((rtimer_clock_t)PACKET_LEN * 35 + 200) * 4;
	tx = 0;

	// wait until the FIFO pin is 1 (i.e., until the first byte is received)
	while (!FIFO_IS_1) {
		if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
			radio_abort_rx();
#if CHAOS_DEBUG
			rx_timeout++;
#endif /* CHAOS_DEBUG */
			return;
		}
	};
#if COOJA
	//OL: do not ask why
	int i;
	for(i = 0; i < 40; i++){
		asm volatile("nop");
	}
#endif /* COOJA */
	// read the first byte (i.e., the len field) from the RXFIFO
	FASTSPI_READ_FIFO_BYTE(CHAOS_LEN_FIELD);
	// keep receiving only if it has the right length
	if (CHAOS_LEN_FIELD != PACKET_LEN) {
		// packet with a wrong length: abort packet reception
		radio_abort_rx();
#if CHAOS_DEBUG
		bad_length++;
#endif /* CHAOS_DEBUG */
		return;
	}
	bytes_read = 1;

#if FINAL_CHAOS_FLOOD
	//Chaos mode on completion
	if( chaos_complete == CHAOS_COMPLETE && tx_cnt_complete < N_TX_COMPLETE){
		tx = 1;
	}
#endif /* FINAL_CHAOS_FLOOD */

	// wait until the FIFO pin is 1 (i.e., until the second byte is received)
	while (!FIFO_IS_1) {
		if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
			radio_abort_rx();
#if CHAOS_DEBUG
			rx_timeout++;
#endif /* CHAOS_DEBUG */
			return;
		}
	};
	// read the second byte (i.e., the header field) from the RXFIFO
	FASTSPI_READ_FIFO_BYTE(CHAOS_HEADER_FIELD);
	// keep receiving only if it has the right header
	if (CHAOS_HEADER_FIELD < CHAOS_HEADER) {
		// packet with a wrong header: abort packet reception
		radio_abort_rx();
#if CHAOS_DEBUG
		bad_header++;
#endif /* CHAOS_DEBUG */
		return;
	}
	bytes_read = 2;
	if (PACKET_LEN > 8) {
		// if packet is longer than 8 bytes, read all bytes but the last 8
		while (bytes_read <= PACKET_LEN - 8) {
			// wait until the FIFO pin is 1 (until one more byte is received)
			while (!FIFO_IS_1) {
				if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
					radio_abort_rx();
#if CHAOS_DEBUG
					rx_timeout++;
#endif /* CHAOS_DEBUG */
					return;
				}
			};
			// read another byte from the RXFIFO
			FASTSPI_READ_FIFO_BYTE(packet[bytes_read]);
			bytes_read++;
		}
	}
}

inline void chaos_end_rx(void) {
	rtimer_clock_t t_rx_stop_tmp = tbccr1;
	if (tx) {
		// packet correctly received and tx required
		if (CHAOS_SYNC_MODE) {
			// increment relay_cnt field
			CHAOS_RELAY_CNT_FIELD++;
		}
		if (tx_cnt == tx_max) {
			// no more Tx to perform: stop Chaos
			radio_off();
			state = CHAOS_STATE_OFF;
		} else {
			// write Chaos packet to the TXFIFO
			if (chaos_complete == CHAOS_COMPLETE) {
				CHAOS_HEADER_FIELD = CHAOS_HEADER+1;
			}
			radio_flush_rx();
			radio_write_tx();
			state = CHAOS_STATE_RECEIVED;
		}
		if (rx_cnt == 0) {
			// first successful reception: store current time
			t_first_rx_l = RTIMER_NOW();
		}

#ifdef LOG_TIRQ
		if (tirq_log_cnt < CHAOS_TIRQ_LOG_SIZE) {
			tirq_log[tirq_log_cnt] = T_irq;
			tirq_log_cnt++;
		}
#endif

		rx_cnt++;
		if (CHAOS_SYNC_MODE && estimate_length && CHAOS_HEADER_FIELD == CHAOS_HEADER) {
			estimate_slot_length(t_rx_stop_tmp);
		}
		t_rx_stop = t_rx_stop_tmp;
		// M: Updated the local data with the currently received data
		*(chaos_data_struct*)data = *(chaos_data_struct*)(&CHAOS_DATA_FIELD);
#if FINAL_CHAOS_FLOOD
		if( chaos_complete == CHAOS_COMPLETE ){
			tx_cnt_complete++;
		}
#endif /* FINAL_CHAOS_FLOOD */
#ifdef LOG_FLAGS
		uint8_t i;
#ifdef LOG_ALL_FLAGS
		if (flags_rx_cnt < CHAOS_FLAGS_LOG_SIZE*MERGE_LEN - MERGE_LEN) {
			relay_counts_rx[flags_rx_cnt / MERGE_LEN] = CHAOS_RELAY_CNT_FIELD;
			for (i = 0; i < MERGE_LEN; i++) {
				flags_rx[flags_rx_cnt] = current_flags_rx[i];
				flags_rx_cnt++;
			}
		}
#else
		// store only the first complete reception
		if (chaos_complete == CHAOS_COMPLETE && flags_rx_cnt == 0) {
			relay_counts_rx[flags_rx_cnt / MERGE_LEN] = CHAOS_RELAY_CNT_FIELD;
			for (i = 0; i < MERGE_LEN-1; i++) {
				flags_rx[flags_rx_cnt] = 0xff;
				flags_rx_cnt++;
			}
			flags_rx[flags_rx_cnt] = CHAOS_COMPLETE_FLAG;
			flags_rx_cnt++;
		}
#endif /* LOG_ALL_FLAGS */
#endif /* LOG_FLAGS */
	} else {
		radio_flush_rx();
		state = CHAOS_STATE_WAITING;
	}
}

inline void chaos_begin_tx(void) {
	SET_PIN_ADC2; //M: To start tx
	t_tx_start = TBCCR1;
	state = CHAOS_STATE_TRANSMITTING;
	tx_relay_cnt_last = CHAOS_RELAY_CNT_FIELD;
	// relay counter to be used in case the timeout expires
	relay_cnt_timeout = CHAOS_RELAY_CNT_FIELD + n_slots_timeout;

	if ((CHAOS_SYNC_MODE) && (T_slot_h) && (!t_ref_l_updated) && (rx_cnt)) {
		// compute the reference time after the first reception (higher accuracy)
		compute_sync_reference_time();
	}
// #ifdef LOG_FLAGS
// #ifdef LOG_ALL_FLAGS
// 	if(tx_activity_cnt<ACTIVITY_LOG_SIZE) {
// 		tx_activity[tx_activity_cnt++] = CHAOS_RELAY_CNT_FIELD;
// 	}
// 	uint8_t i;
// 	if (flags_tx_cnt < CHAOS_FLAGS_LOG_SIZE*MERGE_LEN - MERGE_LEN) {
// 		// M: Store all the 70 history flags. flags_tx_cnt keeps count of next empty space
// 		relay_counts_tx[flags_tx_cnt / MERGE_LEN] = CHAOS_RELAY_CNT_FIELD;
// 		for (i = 0; i < MERGE_LEN; i++) {
// 			flags_tx[flags_tx_cnt] = ((chaos_data_struct *)data)->bv[i];
// 			flags_tx_cnt++;
// 		}
// 	}
// #else
// 	// store only the last transmission
// 	uint8_t i;
// 	flags_tx_cnt = 0;
// 	relay_counts_tx[flags_tx_cnt / MERGE_LEN] = CHAOS_RELAY_CNT_FIELD;
// 	for (i = 0; i < MERGE_LEN; i++) {
// 		flags_tx[flags_tx_cnt] = ((chaos_data_struct *)data)->bv[i];
// 		flags_tx_cnt++;
// 	}
// #endif /* LOG_ALL_FLAGS */
// #endif /* LOG_FLAGS */
}

inline void chaos_end_tx(void) {
	UNSET_PIN_ADC2; //M: To end tx
	ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	t_tx_stop = TBCCR1;
	// stop Chaos if tx_cnt reached tx_max (and tx_max > 1 at the initiator, if sync is enabled)
	if ((++tx_cnt == tx_max) && ((!CHAOS_SYNC_MODE) || ((tx_max - initiator) > 0))) {
		radio_off();
		state = CHAOS_STATE_OFF;
#if FINAL_CHAOS_FLOOD
	} else if ( chaos_complete == CHAOS_COMPLETE && tx_cnt_complete >= N_TX_COMPLETE ){
		radio_off();
		state = CHAOS_STATE_OFF;
#endif /* FINAL_CHAOS_FLOOD */
	} else {
		state = CHAOS_STATE_WAITING;
	}
	radio_flush_tx();
}

/* ------------------------------ Timeouts -------------------------- */
#ifdef LOG_TIRQ
inline void print_tirq(void){
	printf("win_cnt %u, T_slot_tmp %u\n", win_cnt, (uint16_t)(T_slot_h_sum / win_cnt));
	printf("tirq %02x:", tirq_log_cnt);
	uint8_t i;
	for( i = 0; i < tirq_log_cnt; i++ ){
		// printf("%04x,",tirq_log[i] );
		if(tirq_log[i]>34)
			printf("%u,",tirq_log[i]);
	}
	printf("\n");
	tirq_log_cnt = 0;
	memset(&tirq_log, 0, sizeof(tirq_log));
}
#endif

#ifdef LOG_FLAGS
inline void print_flags_tx(void) {
//	uint8_t i;
//	int8_t j;
//	printf("flags_tx %2u:", flags_tx_cnt / MERGE_LEN);
//	for (i = 0; i < flags_tx_cnt / MERGE_LEN; i++) {
//		// For every flag entry, do the following
//		printf("0x%02x-0x%02x-0x", i, relay_counts_tx[i]);
//		for (j = MERGE_LEN-1; j >= 0; j--) {
//			printf("%02x", flags_tx[i*MERGE_LEN + j]);
//		}
//		printf(",");
//	}
//	printf("\n");
	flags_tx_cnt = 0;
	memset(&flags_tx, 0, sizeof(flags_tx));
	memset(&relay_counts_tx, 0, sizeof(relay_counts_tx));
}

inline void print_flags_rx(void) {
//	uint8_t i;
//	int8_t j;
//	printf("flags_rx %2u:", flags_rx_cnt / MERGE_LEN);
//	for (i = 0; i < flags_rx_cnt / MERGE_LEN; i++) {
//		printf("0x%02x-0x%02x-0x", i, relay_counts_rx[i]);
//		for (j = MERGE_LEN-1; j >= 0; j--) {
//			printf("%02x", flags_rx[i*MERGE_LEN + j]);
//		}
//		printf(",");
//	}
//	printf("\n");
	flags_rx_cnt = 0;
	memset(&flags_rx, 0, sizeof(flags_rx));
	memset(&relay_counts_rx, 0, sizeof(relay_counts_rx));
}

inline void print_stats(void) {

	uint8_t i;

	// MM: Prints progress
	printf("Progress %2u: ", solved_cnt);
	int max_cnt = (solved_cnt<CHAOS_SOLVED_LOG_SIZE)?solved_cnt:CHAOS_SOLVED_LOG_SIZE;
	for(i = 0; i < max_cnt; i++) {
		printf("%d ", solve_progress[i]);
		if(solve_progress[i]==CHAOS_NODES)
			break;
	}
	printf("\n");

	// MM: Prints current degree
	printf("Degree %2u: ", solved_cnt);
	for(i = 0; i < max_cnt; i++) {
		printf("%d ", degreelog[i]);
		if(degreelog[i]==(CHAOS_NODES-1))
			break;
	}
	printf("\n");

//	// MM: Prints source
//	printf("Source %2u: ", solved_cnt);
//	for(i = 0; i < max_cnt; i++) {
//		printf("%d ", solve_source[i]);
//	}
//	printf("\n");

//	// MM: Prints decode count
//	printf("Count %2u: ", solved_cnt);
//	for(i = 0; i < max_cnt; i++) {
//		printf("%d ", dclog[i]);
//	}
//	printf("\n");

	solved_cnt = 0;
	memset(&solve_progress, 0, sizeof(solve_progress));
	memset(&solve_source, 0, sizeof(solve_source));
	memset(&dclog, 0, sizeof(dclog));
	memset(&degreelog, 0, sizeof(degreelog));
}
#endif /* LOG_FLAGS */
