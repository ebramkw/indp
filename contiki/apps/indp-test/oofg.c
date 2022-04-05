#include <stdio.h>
#include <stdlib.h>
#include "oofg.h"
#include "indp-test.h"

/* GX = Y */
uint8_t G[COL][MERGE_LEN];

// ebramkw
// saved packets for later decode


int half_decoded_new;

// int wasted_packets;
// int decoded_wasted_packets;
// int discarded_wasted_packets;
// int total_packets;
// int num_not_decoded_packets;
// uint8_t undecoded_stored[UNDECODED_SIZE];
// int undecoded_stored_num;
// uint8_t undecoded_bv[UNDECODED_SIZE][MERGE_LEN];
// char undecoded_payload[UNDECODED_SIZE][PAYLOAD_LEN];
// int decode_count_log[UNDECODED_SIZE];
// int decode_count_log_counter;

// data Matrix
char Y[COL][PAYLOAD_LEN];
#if R_PCA
	#if TOTAL_NODES_RUN
		uint8_t no_update[TOTAL_NODES];		// Bit Vector indicating which node has no update
		uint8_t update_control_bv[TOTAL_MERGE_LEN];
		int update_control_nodes[TOTAL_NODES];
	#else
		uint8_t no_update[COL];
		uint8_t update_control_bv[MERGE_LEN];
		int update_control_nodes[COL];
	#endif
	int num_no_update;
	int nbr_updated;
	#if LOSSY_RPCA
		#if TOTAL_NODES_RUN
			uint8_t data_control[TOTAL_NODES];		// Bit Vector indicating which node has data
			uint8_t data_control_bv[TOTAL_MERGE_LEN];
			int data_control_nodes[TOTAL_NODES];
		#else
			uint8_t data_control[COL];
			uint8_t data_control_bv[MERGE_LEN];
			int data_control_nodes[COL];
		#endif
		int num_data;
		int nbr_data;
		int number_not_decoded;
		int number_not_decoded_2;
		int number_not_decoded_3;
		uint8_t only_once;
		uint8_t prev_not_decoded;
		uint8_t prev_not_decoded_2;
		uint8_t prev_not_decoded_3;
		int update_decoded_new;
		unsigned long target_elapsed[NUM_RECORDS] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500};
		uint8_t reliability[NUM_RECORDS];
		uint8_t reliability_solved[NUM_RECORDS];
		unsigned long reliability_time[NUM_RECORDS];
		unsigned long curr_elapsed;
		// unsigned long num_solved_log[100];
	#endif
#endif

uint8_t CHAOS_BOOTSTRAPPING;

uint8_t pkt_decoded[COL];		// Bit Vector indicating which packet is decoded
int solved[COL];				// List storing the id of the packets decoded already
int num_solved;					// Storing the count of the packets decoded
int nbr_solved;
int nbr_source;
int nbr_destination;

int empty_rows;
int num_ones[COL];

// MM: Nbr solved list
uint8_t nbr_solved_bv_list[NBR_LIST][MERGE_LEN];
int nbr_solved_list[NBR_LIST];
int nbr_common_list[NBR_LIST];
int nbr_source_list[NBR_LIST];
int nbr_relay_list[NBR_LIST];
int nbr_count;
int nbr_list_full;

// MM: Contains the count of latest decoded packets
int decode_count;
int missing_count;
int curr_degree;

// MM: Temporary global
int missing_pkt = -1;
int missing_list[CHAOS_NODES];
int missing_list_ptr[CHAOS_NODES];
int missing_list_count = 0;

// MM: All my neighbors solved, so don't transmit
int neighbor_complete;
int im_complete;
int decoded_new;

// MM: Log distribution
// int encode_counter[COL];

// M: Neighbor info
uint8_t solved_bv[MERGE_LEN];

// ebramkw
// used for debugging
// uint8_t bv[MERGE_LEN];
// uint8_t payload_v[PAYLOAD_LEN];

// M: Soliton Distribution
uint8_t robustsoliton[] = {85, 93, 96, 98, 99, 100}; //C=0.9 D=0.1
int robustsoliton_len = 6;
// uint8_t soliton[] = {5, 52, 68, 77, 83, 87, 89, 91, 93, 94, 95, 96, 97, 98, 99, 100};
// int soliton_len = 16;

// M: Original Growth Code
int origGC[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 4, 5, 6, 7, 10, 15, 15, 15};//Original

// M: New Growth Code
// uint8_t newGC[][31] = { 
//  {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  3,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  2,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  2,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  2,  3,  6,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  2,  3,  7,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  2,  2,  4,  8,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  2,  3,  4,  9,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  2,  2,  3,  5, 10,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  2,  2,  3,  5, 11,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  2,  2,  3,  4,  6, 12,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  2,  2,  3,  4,  6, 13,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  3,  4,  7, 14,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  3,  3,  5,  7, 15,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  3,  4,  5,  8, 16,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  3,  4,  5,  8, 17,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  3,  3,  4,  6,  9, 18,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  3,  3,  4,  6,  9, 19,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  3,  4,  5,  6, 10, 20,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  3,  3,  4,  5,  7, 10, 21,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  4,  5,  7, 11, 22,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  4,  5,  7, 11, 23,  0,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  4,  4,  6,  8, 12, 24,  0,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  4,  5,  6,  8, 12, 25,  0,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  3,  3,  4,  5,  6,  8, 13, 26,  0,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  3,  4,  5,  6,  9, 13, 27,  0,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  3,  3,  4,  4,  5,  7,  9, 14, 28,  0,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  3,  3,  4,  4,  5,  7,  9, 14, 29,  0,  0}, 
//  {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  3,  3,  3,  4,  5,  6,  7, 10, 15, 30,  0}
// };
int8_t newGC[][31] = { 
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0,	 0},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1,	 1},
	{1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2},
	{1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2,	 2},
	{1,	1,	1,	2,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3},
	{1,	1,	1,	1,	2,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3,	 3},
	{1,	1,	1,	1,	2,	2,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4},
	{1,	1,	1,	1,	1,	2,	3,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4,	 4},
	{1,	1,	1,	1,	1,	2,	2,	3,	5,	5,	5,	5,	5,	5,	5,	5,	5,	5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5},
	{1,	1,	1,	1,	1,	1,	2,	2,	3,	5,	5,	5,	5,	5,	5,	5,	5,	5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5,	 5},
	{1,	1,	1,	1,	1,	1,	2,	2,	3,	4,	6,	6,	6,	6,	6,	6,	6,	6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6},
	{1,	1,	1,	1,	1,	1,	1,	2,	2,	3,	4,	6,	6,	6,	6,	6,	6,	6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6,	 6},
	{1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	3,	4,	7,	7,	7,	7,	7,	7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7},
	{1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	3,	3,	5,	7,	7,	7,	7,	7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7,	 7},
	{1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	3,	4,	5,	8,	8,	8,	8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	3,	4,	5,	8,	8,	8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8,	 8},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	3,	3,	4,	6,	9,	9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	3,	3,	4,	6,	9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9,	 9},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	3,	4,	5,	6,	10,	10,	10,	10,	10,	10,	10,	10,	10,	10,	10,	10,	10},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	3,	3,	4,	5,	 7,	10,	10,	10,	10,	10,	10,	10,	10,	10,	10,	10,	10},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	3,	3,	4,	 5,	 7,	11,	11,	11,	11,	11,	11,	11,	11,	11,	11,	11},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	3,	3,	 4,	 5,	 7,	11,	11,	11,	11,	11,	11,	11,	11,	11,	11},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	3,	3,	 4,	 4,	 6,	 8,	12,	12,	12,	12,	12,	12,	12,	12,	12},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	3,	 3,	 4,	 5,	 6,	 8,	12,	12,	12,	12,	12,	12,	12,	12},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	 3,	 3,	 4,	 5,	 6,	 8,	13,	13,	13,	13,	13,	13,	13},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	 3,	 3,	 3,	 4,	 5,	 6,	 9,	13,	13,	13,	13,	13,	13},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	 2,	 3,	 3,	 4,	 4,	 5,	 7,	 9,	14,	14,	14,	14,	14},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	 2,	 2,	 3,	 3,	 4,	 4,	 5,	 7,	 9,	14,	14,	14,	14},
	{1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	 2,	 2,	 3,	 3,	 3,	 4,	 5,	 6,	 7,	10,	15,	15,	15}
};


// TX Condition
//					   { P, 1, P, 1, P, X, P, X };	
int tx_cond[2][2][2] = { 0, 1, 1, 1, 1, 1, 0, 1 };

// // Slot Activity:
// int curr_relay;
// int tx_activity[ACTIVITY_LOG_SIZE];
// int tx_activity_cnt;
// int rx_activity[ACTIVITY_LOG_SIZE];
// int rx_activity_cnt;
// int rxn_activity[ACTIVITY_LOG_SIZE];
// int rxn_activity_cnt;

// // Log Encoding/Decoding Time
// rtimer_clock_t encode_time_log[ACTIVITY_LOG_SIZE];
// rtimer_clock_t decode_time_log[ACTIVITY_LOG_SIZE];
// rtimer_clock_t encode_time;
// rtimer_clock_t decode_time;
// int dp_count;

// // Log help info
// int tx_to_nbr[ACTIVITY_LOG_SIZE][NBR_LIST];
// int rx_from_nbr[ACTIVITY_LOG_SIZE][NBR_LIST];
// int rxn_from_nbr[ACTIVITY_LOG_SIZE][NBR_LIST];

static inline void set_bit(uint8_t *x, int bitNum) {
	// Set the bit
    *x |= (1 << bitNum);
}

static inline void clear_bit(uint8_t *x, int bitNum) {
	// Clear the bit
    *x &= ~(1 << bitNum);
}

uint8_t m_bv[NBR_LIST][MERGE_LEN];
int m_bv_src[NBR_LIST];
int m_bv_cnt;
int m_bv_full;

// ebramkw: pass y vector to encode
void encode(uint8_t* bv, char* yv) {
	// If nothing solved so far, relay whatever is received
	// This would be realised for only the relay nodes initially
	// Others have at least one to begin with
	if(num_solved == 0){
		return;
	}

	uint8_t index, offset;

	// Get the intersection between us: solved_bv and pkt_decoded[]
	// Also compute missing nodes but transmitting just one and removing rest
	int i;
	int intersection = 0;
	memset(missing_list_ptr, 0, CHAOS_NODES * sizeof(int));
	missing_list_count = 0;
	for(i=0;i<CHAOS_NODES;i++) {
		index = i/8;
		if( index >= MERGE_LEN ){
			//out of bound error -> bail out
			continue;
		}
		offset = i%8;
		if(pkt_decoded[i]) {
			if(CHECK_BIT(solved_bv[index],offset)) {
				intersection++;
			} else {
				missing_list[missing_list_count++] = i;
				missing_list_ptr[i] = 1;
			}
		}
	}

	// ebramkw
	// log which packets are solved by my neighbors
	memcpy(nbr_solved_bv_list[nbr_count], solved_bv, MERGE_LEN * sizeof(uint8_t));
	// send with a probability depending on shared between us
	half_decoded_new = 0;
	if((!decoded_new) && num_solved > 0){
		uint8_t nbr_counter_without_my_data = no_neighbor_has_my_data();
		// if no neighbor has the data that I have, always send
		if(nbr_counter_without_my_data == NBR_LIST)
			decoded_new = 1;
		// if some has the data but less than half, send with probability 1/2
		else if((nbr_counter_without_my_data > 0) && (nbr_counter_without_my_data <= (NBR_LIST / 2)))
			half_decoded_new = 1;
		// if more than half neighbors has the data that I have, send with probability = 1/num of neighbors
		else if((nbr_counter_without_my_data > 0) && (nbr_counter_without_my_data > (NBR_LIST / 2)))
			half_decoded_new = nbr_counter_without_my_data;
		else if(nbr_counter_without_my_data == 0)
			half_decoded_new = 0;
	}


	// Log the info shared by nbr in current packet
	#if R_PCA
		nbr_solved_list[nbr_count] = nbr_solved + nbr_updated;
	#else
		nbr_solved_list[nbr_count] = nbr_solved;
	#endif
	nbr_source_list[nbr_count] = nbr_source;
	nbr_count = (nbr_count+1)%NBR_LIST;

	// MM: Process in accordance to the node lagging behind
	int min_nbr_common = find_min(nbr_common_list);
	int min_nbr_solved = find_min(nbr_solved_list);

	// MM: Don't transmit since my neighbors are done
	#if TOTAL_NODES_RUN
		neighbor_complete = (min_nbr_solved == TOTAL_NODES);
	#else
		neighbor_complete = (min_nbr_solved == CHAOS_NODES);
	#endif

	// Clear bv
	memset(bv, 0, MERGE_LEN * sizeof(uint8_t));

#if 1
	// Generate a temporary packet list and initialize it
	int temp_solved[COL];
	int temp_num_solved = 0;
	for(i=0;i<num_solved;i++) {
		int curr_pkt = solved[i];
		if(!missing_list_ptr[curr_pkt]) {
			temp_solved[temp_num_solved++] = curr_pkt;
		}
	}
#else
	// Do this when GC needs to be tested alone - Comment above
	// Generate a temporary packet list and initialize it
    int temp_solved[COL];
    int temp_num_solved;
    memcpy(temp_solved, solved, COL*sizeof(int));
    temp_num_solved = num_solved;
    missing_list_count = 0;
#endif

	// My Growth Code Degree
	#if R_PCA
		#if TOTAL_NODES_RUN
			int degree = 0;
			if((num_solved + num_no_update) < CHAOS_NODES)
				degree = (int)newGC[num_solved + num_no_update][min_nbr_common + num_no_update];
			else
				degree = (int)newGC[num_solved][min_nbr_common];
		#else
			int degree = (int)newGC[num_solved + num_no_update][min_nbr_common + num_no_update];
		#endif
	#else
		int degree = (int)newGC[num_solved][min_nbr_common];
	#endif
	
	if (degree > temp_num_solved) {
		degree = temp_num_solved;
	}
	degree = degree==0?1:degree;
	curr_degree = degree;

	// Soliton Degree
	// int degree;
	// int random_num = ((rand()+32768) % 100);
	// for(degree=0;degree<robustsoliton_len;degree++) {
	// 	if(robustsoliton[degree]>random_num)
	// 		break;
	// }
	// degree++;
	// if (degree > temp_num_solved) {
	// 	degree = temp_num_solved;
	// }
	// // degree = degree==0?1:degree;
	// curr_degree = degree;

	// Orig Growth Code Degree
	// int degree = origGC[min_nbr_solved];
	// if (degree > temp_num_solved) {
	// 	degree = temp_num_solved;
	// }
	// degree = degree==0?1:degree;
	// curr_degree = degree;

	// Number of latest decoded packets to be added	
	decode_count = degree / 3;

	// This wont be modified in case I didn't make use of feedback
	nbr_destination = 0;

	// Start encoding packets
	int temp_solved_index;
	int encode_id;

	// ebramkw
	// Clear yv
	memset(yv, 0, PAYLOAD_LEN * sizeof(char));

	// no more phases
	// if(phase != 2 && phase != 3){
	// 	// Clear yv
	// 	memset(yv, 0, PAYLOAD_LEN * sizeof(char));
	// }

	while(degree) {
		if(missing_list_count) {
			// Computing missing nodes but transmitting just one and removing rest
			// Pick a random missing packet to add to bv
			temp_solved_index = ((rand()+32768) % missing_list_count);
			// Get the packet number
			encode_id = missing_list[temp_solved_index];
			// Add the packet number to maintain distribution
			// encode_counter[encode_id]++;
			// Compute and set the bv
			index = encode_id/8;
			if( index >= MERGE_LEN ){
				//out of bound error -> bail out
				continue;
			}
			offset = encode_id%8;
			bv[index] |= (1 << offset);

			// ebramkw
			//xor Y rows for all bv set to 1 and set result to payload
			for(i = 0; i < curr_payload_len; i++)
				yv[i] ^= Y[encode_id][i];

			//Added one missing packet, unset the flag
			missing_list_count = 0;
			//Add the nbr to which the packet was transmitted
			nbr_destination = nbr_source;
#if 0			
			// Break if only feedback to be tested
			break;
#endif
		} else if (decode_count) {
			// Get the latest packet's number
			encode_id = temp_solved[temp_num_solved-1];
			// Add the packet number to maintain distribution
			// encode_counter[encode_id]++;
			// Compute and set the bv
			index = encode_id/8;
			if( index >= MERGE_LEN ){
				//out of bound error -> bail out
				continue;
			}
			offset = encode_id%8;
			bv[index] |= (1 << offset);

			// ebramkw
			//xor Y rows for all bv set to 1 and set result to payload
			for(i = 0; i < curr_payload_len; i++)
				yv[i] ^= Y[encode_id][i];

			// Delete the packet we just XORed
			temp_num_solved--;
			// Reduce the decode counter by one to add other latest packets
			decode_count--;
		} else {
	 		// Pick a random missing packet to add to bv
			temp_solved_index = ((rand()+32768) % temp_num_solved);
			// Get the packet number
			encode_id = temp_solved[temp_solved_index];
			// Add the packet number to maintain distribution
			// encode_counter[encode_id]++;
			// Compute and set the bv
			index = encode_id/8;
			if( index >= MERGE_LEN ){
				//out of bound error -> bail out
				continue;
			}
			offset = encode_id%8;
			bv[index] |= (1 << offset);

			// ebramkw
			//xor Y rows for all bv set to 1 and set result to payload
			for(i = 0; i < curr_payload_len; i++)
				yv[i] ^= Y[encode_id][i];

			// Delete the packet we just XORed and replace it with the last packet in the array
			temp_solved[temp_solved_index] = temp_solved[temp_num_solved-1];
			temp_num_solved--;
		}
		degree--;
	}

	// // ebramkw
	// int j, k;
	// for (i=0;i<MERGE_LEN;i++) {
	// 	for (j=0;j<8;j++) {
	// 		if(CHECK_BIT(bv[i],j)) {
	// 			for(k = 0; k < PAYLOAD_LEN; k++)
	// 				yv[k] ^= Y[((i*8)+j)][k];
	// 		}
	// 	}
	// }

	// Add logs regarding who was it transmitted for
	// tx_to_nbr[tx_activity_cnt][0] = nbr_destination;

	// Update the solved bv to the transmitted packet
	memset(solved_bv, 0, MERGE_LEN * sizeof(uint8_t));
	for(i=0;i<num_solved;i++) {
		index = solved[i]/8;
		if( index >= MERGE_LEN ){
			//out of bound error -> bail out
			continue;
		}
		offset = solved[i]%8;
		solved_bv[index] |= (1 << offset);
	}

	#if TOTAL_NODES_RUN
		memset(data_control_bv, 0, TOTAL_MERGE_LEN * sizeof(uint8_t));
		for(i=0;i<num_data;i++) {
			index = data_control_nodes[i]/8;
			if( index >= TOTAL_MERGE_LEN ){
				//out of bound error -> bail out
				continue;
			}
			offset = data_control_nodes[i]%8;
			data_control_bv[index] |= (1 << offset);
		}

		memset(update_control_bv, 0, TOTAL_MERGE_LEN * sizeof(uint8_t));
		for(i=0;i<num_no_update;i++) {
			index = update_control_nodes[i]/8;
			if( index >= TOTAL_MERGE_LEN ){
				//out of bound error -> bail out
				continue;
			}
			offset = update_control_nodes[i]%8;
			update_control_bv[index] |= (1 << offset);
		}
	#else
		memset(data_control_bv, 0, MERGE_LEN * sizeof(uint8_t));
		for(i=0;i<num_data;i++) {
			index = data_control_nodes[i]/8;
			if( index >= MERGE_LEN ){
				//out of bound error -> bail out
				continue;
			}
			offset = data_control_nodes[i]%8;
			data_control_bv[index] |= (1 << offset);
		}

		memset(update_control_bv, 0, MERGE_LEN * sizeof(uint8_t));
		for(i=0;i<num_no_update;i++) {
			index = update_control_nodes[i]/8;
			if( index >= MERGE_LEN ){
				//out of bound error -> bail out
				continue;
			}
			offset = update_control_nodes[i]%8;
			update_control_bv[index] |= (1 << offset);
		}
	#endif
}

// ebramkw: pass y vector to decode and no_update to update current
#if R_PCA
	#if LOSSY_RPCA
		int decode(uint8_t* bv, uint8_t* solved_bv, char* yv, uint8_t* no_update_bv, uint8_t* data_bv) {
	#else
		int decode(uint8_t* bv, uint8_t* solved_bv, char* yv, uint8_t* no_update_bv) {
	#endif
#else
	int decode(uint8_t* bv, uint8_t* solved_bv, char* yv) {
#endif
	/* local variables */
	uint8_t i;							// Counter
	uint8_t s, s_index, s_offset;	// Index of the leftmost 1
	int eq_ones;					// Degree of the current equation
	int prev_solved = num_solved;	// Num solved until now
	uint8_t index, offset;

	// Stores the number of packets decoded now
	decode_count = 0;

	// ebramkw: cancel optimization to read all the equation to edit the data accordingly
	// Optimization
	// for(i=0;i<COL;i++) {
	// 	if(pkt_decoded[i]) {
	// 		bv[i/8] &= ~(1 << (i%8));
	// 	}
	// }

	// total number of received packets in a round
	// total_packets++;

	// get number of wasted packets
	// packet is wasted if I cannot decode it
	// I cannot decode it if I don't have more than one not known to me
	//***
	//***add a pkt_decoded bv instead of using pkt_decoded array***
	//***
	// int num_ones_I_dont_have = 0;
	// uint8_t index, offset;
	// for(i = 0; i < COL; i++){
	// 	index = i / 8;
	// 	if( index >= MERGE_LEN ){
	// 		//out of bound error -> bail out
	// 		continue;
	// 	}
	// 	offset = i % 8;

	// 	if(CHECK_BIT(bv[index], offset) && !pkt_decoded[i])
	// 		num_ones_I_dont_have++;
	// 	if(num_ones_I_dont_have > 1)
	// 		break;
	// }
	// // should keep it for later decoding!
	// if(num_ones_I_dont_have > 1){
	// 	// total wasted in a round
	// 	wasted_packets++;

	// 	// store packet (bv and yv)
	// 	undecoded_stored[undecoded_stored_num] = 1;
	// 	memcpy(undecoded_bv[undecoded_stored_num], bv, MERGE_LEN * sizeof(uint8_t));
	// 	memcpy(undecoded_payload[undecoded_stored_num], yv, PAYLOAD_LEN * sizeof(char));
	// 	// used for ref to arrays
	// 	undecoded_stored_num = (undecoded_stored_num + 1) % UNDECODED_SIZE;

	// 	// current num
	// 	num_not_decoded_packets++;
	// }

	// Get the index and offset of leftmost one
	s = leftmost_one(bv);s_index = s/8;s_offset = s%8;
	
	// Find the degree of new equation
	eq_ones = degree_eq(bv);

	while((eq_ones > 0) && CHECK_BIT(G[s][s_index],s_offset)) {
		if(eq_ones >= num_ones[s]) {
			xor_bv_matrix(bv,s);

			// ebramkw
			// newY = newY (received payload) xor row s of matrix Y
			xor_yv_matrix(yv,s);
			
			// if(phase != 2 && phase != 3){
			// 	// newY = newY (received payload) xor row s of matrix Y
			// 	xor_yv_matrix(yv,s);
			// }
		} else {			
			swap_eq(bv, s);

			// ebramkw
			// swap newY with row s of matrix Y
			swap_code(yv, s);

			// if(phase != 2 && phase != 3){
			// 	// swap newY with row s of matrix Y
			// 	swap_code(yv, s);
			// }

			num_ones[s] = eq_ones;
			substitution(s, bv, yv);
		}
		// Get the index and offset of leftmost one
		s = leftmost_one(bv);s_index = s/8;s_offset = s%8;
		// Find the degree of new equation
		eq_ones = degree_eq(bv);
	}
	if(eq_ones > 0) {
		copy_eq(bv, s);

		// ebramkw
		// set row s of matrix Y to newY
		copy_code(yv, s);
		
		// if(phase != 2 && phase != 3){
		// 	// set row s of matrix Y to newY
		// 	copy_code(yv, s);
		// }

		num_ones[s] = eq_ones;
		empty_rows--;
		substitution(s, bv, yv);
	}

	#if R_PCA
		nbr_updated = 0;
		#if LOSSY_RPCA
			nbr_data = 0;
			int decoded_once = 0;
			int decoded_once_data_control = 0;
			int decoded_once_data = 0;
			update_decoded_new = 0;
		#endif
	#endif
	// Set decode_count, pkt_decoded, solved, num_solved, decoded_new
	decoded_new = 0;
	// solved_new, only if new data is solved
	int solved_new = 0;
	// uint8_t index, offset;
	nbr_solved = 0;
	for(i=0;i<COL;i++) {
		if((!pkt_decoded[i])&&(num_ones[i]==1)) {
			// should happen only once for all node except sender should not happen
			decode_count++;
			pkt_decoded[i] = 1;
			solved[num_solved++] = i;
			decoded_new = 1;
			solved_new = 1;
			#if LOSSY_RPCA
				decoded_once = 1;
				decoded_once_data = 1;
				// cannot happen
				// if packed is solved and not marked in data_control ==> mark it
				// if(!data_control[i]){
				// 	data_control[i] = 1;
				// 	num_data++;
				// }
			#endif
		}
		index = i / 8;
		if( index >= MERGE_LEN ){
			//out of bound error -> bail out
			continue;
		}
		offset = i % 8;
		// update nbr_solved from solved_bv instead of sending it on the packet
		if(CHECK_BIT(solved_bv[index],offset))
			nbr_solved++;

		// source must deliver to his neighbors
		// I have data but my neighbor didn't get it
		// if(!decoded_new && (i == node_index) && pkt_decoded[node_index] && !CHECK_BIT(solved_bv[index],offset))
		// 	decoded_new = 1;

		#if R_PCA
			// update current no_update from received no_update
			if(CHECK_BIT(no_update_bv[index],offset)){
				// get how much nbr updated
				nbr_updated++;
				if(no_update[i] == 0){
					no_update[i] = 1;
					update_control_nodes[num_no_update++] = i;

					
					// #if !LOSSY_RPCA
						decoded_new = 1;
					// #else
					// 	update_decoded_new = 1;
					// #endif
					
					
					// if(num_solved > 0 || num_data > 0)
					// 	decoded_new = 1;
				}
			}
			/*else if(no_update[i]) {
				no_update_bv[index] |= (1 << offset);
			}*/

			#if LOSSY_RPCA
				// update current data_control from received data_control_bv
				if(CHECK_BIT(data_bv[index],offset)){
					// get how much nbr updated
					nbr_data++;
					if(data_control[i] == 0){
						// why I found it more than one for one sender!!!
						// should happen only once for all node except sender should not happen
						data_control[i] = 1;
						data_control_nodes[num_data++] = i;
						

						// #if !LOSSY_RPCA
							decoded_new = 1;
						// #else
						// 	update_decoded_new = 1;
						// #endif
						

						decoded_once = 1;
						decoded_once_data_control = 1;
					}
				}
				/*else if(data_control[i]) {
					data_bv[index] |= (1 << offset);
				}*/
			#endif
		#endif
	}

	// s = leftmost_one(no_update_bv);s_index = s/8;s_offset = s%8;
	// eq_ones = degree_eq(no_update_bv);
	// while(eq_ones > 0){
	// 	nbr_updated++;

	// 	if(no_update[i] == 0){
	// 		no_update[i] = 1;
	// 		update_control_nodes[num_no_update++] = i;
	// 		#if !LOSSY_RPCA
	// 			decoded_new = 1;
	// 		#else
	// 			update_decoded_new = 1;
	// 		#endif
	// 	}

	// 	s = leftmost_one(no_update_bv);s_index = s/8;s_offset = s%8;
	// 	eq_ones = degree_eq(no_update_bv);
	// }

	// s = leftmost_one(data_bv);s_index = s/8;s_offset = s%8;
	// eq_ones = degree_eq(data_bv);
	// while(eq_ones > 0){
	// 	nbr_data++;

	// 	if(data_control[i] == 0){
	// 		data_control[i] = 1;
	// 		data_control_nodes[num_data++] = i;
	// 		decoded_new = 1;
	// 		decoded_once = 1;
	// 		decoded_once_data_control = 1;
	// 	}

	// 	s = leftmost_one(data_bv);s_index = s/8;s_offset = s%8;
	// 	eq_ones = degree_eq(data_bv);
	// }

	// if(decode_count != 0 && decode_count_log_counter < UNDECODED_SIZE){
	// 	decode_count_log[decode_count_log_counter++] = decode_count;
	// }


	// #if TOTAL_NODES_RUN
	// 	for(i = CHAOS_NODES; i < TOTAL_NODES; i++){
	// 		index = i / 8;
	// 		if( index >= TOTAL_MERGE_LEN ){
	// 			//out of bound error -> bail out
	// 			continue;
	// 		}
	// 		offset = i % 8;
	// 		#if R_PCA
	// 			// update current no_update from received no_update
	// 			if(CHECK_BIT(no_update_bv[index],offset)){
	// 				// get how much nbr updated
	// 				nbr_updated++;
	// 				if(no_update[i] == 0){
	// 					no_update[i] = 1;
	// 					update_control_nodes[num_no_update++] = i;
	// 					#if !LOSSY_RPCA
	// 						decoded_new = 1;
	// 					#else
	// 						update_decoded_new = 1;
	// 					#endif
	// 					// if(num_solved > 0 || num_data > 0)
	// 					// 	decoded_new = 1;
	// 				}
	// 			}
	// 			/*else if(no_update[i]) {
	// 				no_update_bv[index] |= (1 << offset);
	// 			}*/

	// 			#if LOSSY_RPCA
	// 				// update current data_control from received data_control_bv
	// 				if(CHECK_BIT(data_bv[index],offset)){
	// 					// get how much nbr updated
	// 					nbr_data++;
	// 					if(data_control[i] == 0){
	// 						data_control[i] = 1;
	// 						data_control_nodes[num_data++] = i;
	// 						decoded_new = 1;
	// 						decoded_once = 1;
	// 						decoded_once_data_control = 1;
	// 					}
	// 				}
	// 				/*else if(data_control[i]) {
	// 					data_bv[index] |= (1 << offset);
	// 				}*/
	// 			#endif
	// 		#endif
	// 	}
	// #endif

	#if R_PCA
		#if LOSSY_RPCA
			// not decoded new data or new data_control for two consecutive packets
			if((!decoded_once) && prev_not_decoded){
				number_not_decoded++;
			} else{
				number_not_decoded = 0;
			}

			if((!decoded_once)){
				prev_not_decoded = 1;
			} else{
				prev_not_decoded = 0;
			}

			if(num_solved < 1){
				if((!decoded_once_data) && prev_not_decoded_2){
					number_not_decoded_2++;
				} else{
					number_not_decoded_2 = 0;
				}

				if((!decoded_once_data)){
					prev_not_decoded_2 = 1;
				} else{
					prev_not_decoded_2 = 0;
				}
			}

			if(num_data < 1){
				if((!decoded_once_data_control) && prev_not_decoded_3){
					number_not_decoded_3++;
				} else{
					number_not_decoded_3 = 0;
				}

				if((!decoded_once_data_control)){
					prev_not_decoded_3 = 1;
				} else{
					prev_not_decoded_3 = 0;
				}
			}

			// // check if I decoded new info after I'm done
			// if(im_complete && decoded_once){
			// 	im_complete = 0;
			// 	number_not_decoded = 0;
			// 	prev_not_decoded = 0;
			// 	only_once = 0;
			// 	num_undo++;
			// 	slots = 0;
			// 	ticks = 0;
			// }
		#endif
	#endif

	// if I have not decoded packets
	/*if(num_not_decoded_packets){
		// check if I can decode now, remove from not_decoded_packets and reduce num_not_decoded_packets
		// compare current decoded packets with stored not_decoded bv
		for(i = 0; i < UNDECODED_SIZE; i++){
			if(undecoded_stored[i]){
				num_ones_I_dont_have = 0;
				for(j = 0; j < COL; j++){
					index = j / 8;
					if( index >= MERGE_LEN ){
						//out of bound error -> bail out
						continue;
					}
					offset = j % 8;

					if(CHECK_BIT(undecoded_bv[i][index], offset) && !pkt_decoded[j])
						num_ones_I_dont_have++;
					
					if(num_ones_I_dont_have > 1)
						break;
				}

				// cannot decode, num_ones_I_dont_have > 1 ==> don't care

				// already decoded, no new info
				if(num_ones_I_dont_have == 0){
					undecoded_stored[i] = 0;
					num_not_decoded_packets--;
					discarded_wasted_packets++;
				} else if(num_ones_I_dont_have == 1){
					// can decode
					undecoded_stored[i] = 0;
					num_not_decoded_packets--;
					decoded_wasted_packets++;

					s = leftmost_one(undecoded_bv[i]);s_index = s/8;s_offset = s%8;
					// Find the degree of new equation
					eq_ones = degree_eq(undecoded_bv[i]);

					while((eq_ones > 0) && CHECK_BIT(G[s][s_index],s_offset)) {
						if(eq_ones >= num_ones[s]) {
							xor_bv_matrix(undecoded_bv[i],s);

							// ebramkw
							// newY = newY (received payload) xor row s of matrix Y
							xor_yv_matrix(undecoded_payload[i],s);
							
							// if(phase != 2 && phase != 3){
							// 	// newY = newY (received payload) xor row s of matrix Y
							// 	xor_yv_matrix(yv,s);
							// }
						} else {			
							swap_eq(undecoded_bv[i], s);

							// ebramkw
							// swap newY with row s of matrix Y
							swap_code(undecoded_payload[i], s);

							// if(phase != 2 && phase != 3){
							// 	// swap newY with row s of matrix Y
							// 	swap_code(yv, s);
							// }

							num_ones[s] = eq_ones;
							substitution(s, undecoded_bv[i], undecoded_payload[i]);
						}
						// Get the index and offset of leftmost one
						s = leftmost_one(undecoded_bv[i]);s_index = s/8;s_offset = s%8;
						// Find the degree of new equation
						eq_ones = degree_eq(undecoded_bv[i]);
					}
					if(eq_ones > 0) {
						copy_eq(undecoded_bv[i], s);

						// ebramkw
						// set row s of matrix Y to newY
						copy_code(undecoded_payload[i], s);
						
						// if(phase != 2 && phase != 3){
						// 	// set row s of matrix Y to newY
						// 	copy_code(yv, s);
						// }

						num_ones[s] = eq_ones;
						empty_rows--;
						substitution(s, undecoded_bv[i], undecoded_payload[i]);
					}
					for(k=0;k<COL;k++) {
						if((!pkt_decoded[k])&&(num_ones[k]==1)) {
							// should happen only once for all node except sender should not happen
							decode_count++;
							pkt_decoded[k] = 1;
							solved[num_solved++] = k;
							decoded_new = 1;

							// if(undecoded_stored[i]){
							// 	decoded_wasted_packets++;
							// 	undecoded_stored[i] = 0;
							// 	num_not_decoded_packets--;
							// }
						}
					}
				}
			}

			if(num_not_decoded_packets == 0)
				break;
		}
	}*/

	// if(decoded_new) {
	// 	if(rxn_activity_cnt<ACTIVITY_LOG_SIZE) {
	// 		rxn_from_nbr[rxn_activity_cnt][0] = nbr_destination;
	// 		rxn_activity[rxn_activity_cnt++] = curr_relay;
	// 	}
	// }
	// else {
	// 	if(rx_activity_cnt<ACTIVITY_LOG_SIZE) {
	// 		rx_from_nbr[rx_activity_cnt][0] = nbr_destination;
	// 		rx_activity[rx_activity_cnt++] = curr_relay;
	// 	}
	// }

	// Return 1 if we have solved
	// if(num_solved == CHAOS_NODES) {
	// 	return 1;
	// }
	// 
	// return 0;

	#if R_PCA
		#if LOSSY_RPCA
			if((!CHAOS_BOOTSTRAPPING) && (!just_started)){
				// current elapsed
				unsigned long total_ticks = ((3 * curr_slot) * (RTIMER_SECOND / 2)) + curr_elapsed;
				unsigned long current_elapsed = (total_ticks * 1e3) / RTIMER_SECOND;

				// record reliability in different times
				if(current_elapsed < target_elapsed[2]){
					reliability[1] = num_data + num_no_update;
					reliability_time[1] = current_elapsed;
					reliability_solved[1] = num_solved;
				} else if(current_elapsed >= target_elapsed[(NUM_RECORDS - 1)]){
					reliability[(NUM_RECORDS - 1)] = num_data + num_no_update;
					reliability_time[(NUM_RECORDS - 1)] = current_elapsed;
					reliability_solved[(NUM_RECORDS - 1)] = num_solved;
				} else{
					for(i = 1; i < (NUM_RECORDS - 1); i++){
						if((current_elapsed >= target_elapsed[i]) && (current_elapsed < target_elapsed[i + 1])){
							reliability[i] = num_data + num_no_update;
							reliability_time[i] = current_elapsed;
							reliability_solved[i] = num_solved;
							break;
						}
					}
				}

				if((control_total_secs == 0) && (num_no_update + num_data) == CHAOS_NODES)
					control_total_secs = current_elapsed;

				#if ONE_SRC
				if((one_total_secs != 100) && (one_total_secs == 0) && (num_solved == 1))
					one_total_secs = current_elapsed;
				#endif
			}

			#if TOTAL_NODES_RUN
				return ((num_solved + num_no_update) == TOTAL_NODES);
			#else
				return ((num_solved + num_no_update) == CHAOS_NODES);
			#endif
		#else
			return ((num_solved + num_no_update) == CHAOS_NODES);
		#endif
	#else
		return (num_solved == CHAOS_NODES);
	#endif
}

// ebramkw: pass y vector
void substitution(uint8_t s, uint8_t* bv, char* yv) {
	uint8_t j, k;
	uint8_t s_index = s/8;
	uint8_t s_offset = s%8;
	for(j=0;j<s;j++) {
		if(CHECK_BIT(G[j][s_index],s_offset)) {
			xor_matrix_bv(j,bv);
			
			// ebramkw
			// edit the y vector
			xor_matrix_yv(j,yv);
			
			// if(phase != 2 && phase != 3){
			// 	// edit the y vector
			// 	xor_matrix_yv(j,yv);
			// }

			num_ones[j] = degree_eq(G[j]);
		}
	}
	for(j=s+1;j<COL;j++) {
		for(k=0;k<j;k++) {
			if(CHECK_BIT(G[j][j/8],j%8) && CHECK_BIT(G[k][j/8],j%8)) {
				xor_matrix_matrix(k,j);
				
				// ebramkw
				// edit the Y Matrix
				xor_matrix_matrix_Y(k,j);
				
				// if(phase != 2 && phase != 3){
				// 	// edit the Y Matrix
				// 	xor_matrix_matrix_Y(k,j);
				// }

				num_ones[k] = degree_eq(G[k]);
			}
		}
	}
}

int find_min(int list[]) {
	int i;
	int min_solved = 100;
	for(i=0;i<NBR_LIST;i++)
		if(nbr_solved_list[i]<min_solved)
			min_solved = nbr_solved_list[i];
	if(min_solved==100)
		return 0;	
	return min_solved;
}

uint8_t leftmost_one(uint8_t* bv) {
	int i, j;
	for (i=0;i<MERGE_LEN;i++) {
		for (j=0;j<8;j++) {
			if(((i*8)+j)<CHAOS_NODES && CHECK_BIT(bv[i],j)) {
				return ((i*8)+j);
			}
		}
	}
	return 0;
}

uint8_t degree_eq(uint8_t* bv) {
	int i;
	uint8_t c, v, degree = 0;
	for (i=0;i<MERGE_LEN;i++) {
		v = bv[i];
		for (c = 0; v; v >>= 1)
			c += v & 1;
		degree += c;
	}
	return degree;
}

void xor_bv_matrix(uint8_t* bv, uint8_t s) {
	int i;
	for(i=0;i<MERGE_LEN;i++) {
		bv[i] ^= G[s][i];
	}
}

// ebramkw
void xor_yv_matrix(char* yv, uint8_t s) {
	int i;
	for(i=0;i<curr_payload_len;i++) {
		yv[i] ^= Y[s][i];
	}
}


void xor_matrix_bv(uint8_t s, uint8_t* bv) {
	int i;
	for(i=0;i<MERGE_LEN;i++) {
		G[s][i] ^= bv[i]; 
	}
}

// ebramkw
void xor_matrix_yv(uint8_t s, char* yv) {
	int i;
	for(i=0;i<curr_payload_len;i++) {
		Y[s][i] ^= yv[i]; 
	}
}


void xor_matrix_matrix(uint8_t s, uint8_t r) {
	int i;
	for(i=0;i<MERGE_LEN;i++) {
		G[s][i] ^= G[r][i]; 
	}
}

// ebramkw
void xor_matrix_matrix_Y(uint8_t s, uint8_t r) {
	int i;
	for(i=0;i<curr_payload_len;i++) {
		Y[s][i] ^= Y[r][i]; 
	}
}


void swap_eq(uint8_t* bv, uint8_t s) {
	int i;
	uint8_t temp;
	for(i=0;i<MERGE_LEN;i++) {
		temp = bv[i];
		bv[i] = G[s][i];
		G[s][i] = temp;
	}
}

// ebramkw
void swap_code(char* yv, uint8_t s) {
	int i;
	char temp;
	for(i=0;i<curr_payload_len;i++) {
		temp = yv[i];
		yv[i] = Y[s][i];
		Y[s][i] = temp;
	}
}


void copy_eq(uint8_t* bv, uint8_t s) {
	int i;
	for(i=0;i<MERGE_LEN;i++) {
		G[s][i] = bv[i];
	}
}

// ebramkw
void copy_code(char* yv, uint8_t s) {
	int i;
	for(i=0;i<curr_payload_len;i++) {
		Y[s][i] = yv[i];
	}
}


void print_uint8_t(uint8_t bv) {
	uint8_t i;
	for(i=0;i<8;i++) {
		if(CHECK_BIT(bv,i))
			printf("1");
		else
			printf("0");
	}
}

// void get_projected_data(){
// 	int i, j;
// 	for (i = 0; i < CHAOS_NODES; i++){
// 		// if(i != node_index){
// 			for(j = 0; j < PAYLOAD_LEN/4; j++){
// 				total_projected_data_matrix[i][j + ((data_round - 1) * (PAYLOAD_LEN/4))][0] = (*(float*)&Y[i][j * 4]);
// 			}
// 		// }
// 	}

// 	// int c;
// 	// printf("node %d: round %d, last 3 values %ld %ld %ld\n", node_id, data_round, (long)(*(float*)&Y[3][2 * 4]), (long)(*(float*)&Y[3][3 * 4]), (long)(*(float*)&Y[3][4 * 4]));
// 	// for(i = 0; i < CHAOS_NODES; i++)
// 	// {
// 	// 	for(c = 0; c < M; c++){
// 	// 		printf("node %d: total projected of node%d row %d = %ld\n", node_id, (i+1), c, (long)total_projected_data_matrix[i][c][0]);
// 	// 	}
// 	// }
// }

void global_pca() {
	// SVD
	// ebramkw
	// local computation after codecast is done
	// global PCA based on SVD

	// total approximated data
	float approximated_data_matrix[CHAOS_NODES * SHARE_SV][N];

	float global_singular_values[N];
	
	// concatenate eigenvalues * eigenvectors to build data matrix
	int i = 0, c, d;
	int dec;
	float frac;
	while(i < CHAOS_NODES){
		// get approximated data for each and concatenate it
		
		// *** for the time, will use one PCA *** //
		// get singular values matrix
		// get singular vectors transpose matrix
		// get approximated data
		// concatenate in total matrix

		for (c = 0; c < N; c++) {
			approximated_data_matrix[(i * SHARE_SV)][c] = 0;
			for (d = 0; d < SHARE_SV; d++) {
				approximated_data_matrix[(i * SHARE_SV)][c] += (*(float*)&Y[i][(d * 4 * (N + 1))]) * (*(float*)&Y[i][(((c + 1)  * 4) + (d * 4 * (N + 1)))]);

				// printf("node %d: app[%d][%d] = Y[%d][%d] * Y[%d][%d]", node_id, (i * SHARE_SV), c, i, (d * 4 * (N + 1)), i, (((c + 1)  * 4) + (d * 4 * (N + 1))));
				
				// dec = (*(float*)&Y[i][(d * 4 * (N + 1))]);
				// frac = (*(float*)&Y[i][(d * 4 * (N + 1))]) - dec;
				// if((*(float*)&Y[i][(d * 4 * (N + 1))]) < 0){
				// 	//dec *= -1;
				// 	frac = (*(float*)&Y[i][(d * 4 * (N + 1))]) * -1;
				// 	frac = frac + dec;
				// }
				// printf("%d.%02u * ", dec, (unsigned int)(frac * 100));

				// dec = (*(float*)&Y[i][(((c + 1)  * 4) + (d * 4 * (N + 1)))]);
				// frac = (*(float*)&Y[i][(((c + 1)  * 4) + (d * 4 * (N + 1)))]) - dec;
				// if((*(float*)&Y[i][(((c + 1)  * 4) + (d * 4 * (N + 1)))]) < 0){
				// 	//dec *= -1;
				// 	frac = (*(float*)&Y[i][(((c + 1)  * 4) + (d * 4 * (N + 1)))]) * -1;
				// 	frac = frac + dec;
				// }
				// printf("%d.%02u\n", dec, (unsigned int)(frac * 100));
			}
		}
		i++;
	}

	// for (i = 0; i < CHAOS_NODES * SHARE_SV; i++)
	// {
	// 	printf("node %d: FINAL MATRIX", node_id);
	// 	for (c = 0; c < N; c++)
	// 	{
	// 		dec = approximated_data_matrix[i][c];
	// 		frac = approximated_data_matrix[i][c] - dec;
	// 		if(approximated_data_matrix[i][c] < 0){
	// 			//dec *= -1;
	// 			frac = approximated_data_matrix[i][c] * -1;
	// 			frac = frac + dec;
	// 		}
	// 		printf("%d.%02u ", dec, (unsigned int)(frac * 100));
	// 	}
	// 	printf("\n");
	// }

	// svd on data matrix
	float global_singular_vectors[N][N];
	i = dsvd(approximated_data_matrix, CHAOS_NODES * SHARE_SV, N, global_singular_values, global_singular_vectors);

	Sort_by_Decreasing_Singular_Values(N, global_singular_values, global_singular_vectors);
	
	for(c = 0; c < N; c++)
		global_singular_vector[c] = global_singular_vectors[c][0];
	
	// printing global eigenvalue and eigenvector
	/*if(i == 1) {
		// print eigenvalue and eigenvaector
		int dec;
		float frac;
		dec = global_singular_values[0];
		frac = global_singular_values[0] - dec;
		if(global_singular_values[0] < 0){
			//dec *= -1;
			frac = global_singular_values[0] * -1;
			frac = frac + dec;
		}
		printf("node %d: global singular value is %d.%04u ", node_id, dec, (unsigned int)(frac * 10000));

		// printf("node %d: global pca is %d ", node_id, (int)global_singular_values[0]);
		
		printf("node %d: the global eigen vector is : ", node_id);
		for(i = 0; i < N; i++){
			dec = global_singular_vector[i];
			frac = global_singular_vector[i] - dec;
			if(global_singular_vector[i] < 0){
				frac = global_singular_vector[i] * -1;
				frac = frac + dec;
			}
			printf("%d.%04u ", dec, (unsigned int)(frac * 10000));

			// printf("%d ", (int)global_singular_vector[i]);
		}
		printf("\n");
	}*/


	// // PIM
	// // ebramkw
	// // local computation after codecast is done
	// // global PCA based on PIM
	// float v_local[N];
	// uint8_t c, d, j;
	// float covariance_M_sum[N][N];
	// memset(covariance_M_sum, 0, N * N * sizeof(float));

	// int i = 0;
	// while(i < CHAOS_NODES){
	// 	for (c = 0; c < N; c++) {
	// 		v_local[c] = 0;
	// 		for (d = 0; d < SHARE_SV; d++) {
	// 			v_local[c] += (*(float*)&Y[i][(d * 4 * (N + 1))]) * (*(float*)&Y[i][(((c + 1)  * 4) + (d * 4 * (N + 1)))]);
	// 		}
	// 	}

	// 	for(c = 0; c < N; c++) {
	// 		for (d = 0; d < N; d++) {
	// 			covariance_M_sum[c][d] += v_local[c] * v_local[d];
	// 		}
	// 	}
	//  	i++;
	// }

	// //decompose eigenvectors from final covariance matrix
	// float zmax, emax;
	// float x[N], e[N], z[N];
	// x[0] = 1; x[1] = 0; x[2] = 0;
	// c = 0;
	// do {
	// 	for(i = 0; i < N; i++){
 //            z[i]=0;
 //            for(j = 0; j < N; j++)
 //            {
 //                z[i] = z[i] + covariance_M_sum[i][j] * x[j];
 //            }
 //        }

 //        zmax = fabs(z[0]);
 //        for(i = 1; i < N; i++){
 //            if((fabs(z[i])) > zmax)
 //                zmax = fabs(z[i]);
 //        }
 //        for(i = 0; i < N; i++){
 //            z[i] = z[i] / zmax;
 //        }

 //        for(i = 0; i < N; i++){
 //            e[i] = 0;
 //            e[i] = fabs((fabs(z[i])) - (fabs(x[i])));
 //        }

 //        emax = e[0];
 //        for(i = 1; i < N; i++){
 //            if(e[i] > emax)
 //                emax = e[i];
 //        }
 //        for(i = 0; i < N; i++){
 //            x[i] = z[i];
 //        }
 //        c++;
	// } while(emax > 0.001);

	// int dec;
	// float frac;
	// dec = zmax;
	// frac = zmax - dec;
 //    printf("node %d, takes %d: the required eigen value is %d.%04u\n", node_id, c, dec, (unsigned int)(frac * 10000));
 //    printf("node %d, the required eigen vector is : ", node_id);
 //    for(i = 0; i < N; i++){
 //    	dec = z[i];
	// 	frac = z[i] - dec;
 //        printf("%d.%04u ", dec, (unsigned int)(frac * 10000));
 //    }
 //    printf("\n");
}

void print_g() {
	// ebramkw: print data matrix
	int i, j, dec;
	float frac, value;
	for(i = 0; i < CHAOS_NODES; i++) {
		#if R_PCA
			printf("node %d, update %d, row %d: ", node_id, no_update[i], i);
		#else
			printf("node %d, row %d: ", node_id, i);
		#endif
		
		for(j = 0; j < PAYLOAD_LEN; j+=4){
			value = *(float*)&Y[i][j];
			dec = value;
			frac = value - dec;
			// if(value < 0){
			// 	//dec *= -1;
			// 	frac = value * -1;
			// 	frac = frac + dec;
			// 	printf("-%d.%02u ", dec, (unsigned int)(frac * 100));
			// } else
			printf("%d.%02u ", dec, (unsigned int)(frac * 100));
		}
		printf("\n");
	}

	
	#if DEBUG
	int i;
	#endif

	#if DEBUG
	printf("Printing G\n");
	for(i=0;i<COL;i++) {
		for(j=0;j<MERGE_LEN;j++) {
			print_uint8_t(G[i][j]);
		}
		printf("\n");
	}
	#endif

	#if DEBUG
	printf("NumOnes: ");
	for(i=0;i<COL;i++) {
		printf("%d ",num_ones[i]);
	}
	printf("\n");
	#endif

	#if DEBUG
	Print distribution
	printf("Distribution: ");
	for(i=0;i<COL;i++) {
		printf("%d ", encode_counter[i]);
	}
	printf("\n");
	#endif

	#if DEBUG
	printf("node %d, NbrList(%d): ", node_id, NBR_LIST);
	for(i=0;i<NBR_LIST;i++) {
		printf("%d ", nbr_solved_list[i]);
	}
	printf("\n");
	#endif
}

// ebramkw: convert float to array of bytes
void float2Bytes(float val, char* bytes_array){
	// Create union of shared memory space
	union {
		float float_variable;
		char temp_array[4];
	} u;
	// Overite bytes of union with float variable
	u.float_variable = val;
	// Assign bytes to input array
	memcpy(bytes_array, u.temp_array, 4);
}

// uint8_t send_data(int node_index){
// 	int i;
// 	for(i = 0; i < NODES_SEND_DATA; i++){
// 		if(nodes[(0 + i) % CHAOS_NODES] == node_index)
// 			return 1;
// 	}
// 	return 0;

// 	// fixed 4 nodes
// 	// if(node_index == 1 || node_index == 2 || node_index == 3 || node_index == 4)
// 	// 	return 1;
// 	// return 0;
// }

uint8_t no_neighbor_has_my_data(){
	uint8_t i, j;
	uint8_t index, offset;
	uint8_t nbr_counter = 0;
	for(i = 0; i < NBR_LIST; i++){
		for(j = 0; j < CHAOS_NODES; j++){
			index = j/8;
			if(index >= MERGE_LEN){
				//out of bound error -> bail out
				continue;
			}
			offset = j%8;
			if(pkt_decoded[j] && !CHECK_BIT(nbr_solved_bv_list[i][index],offset)){
				nbr_counter++;
				break;
			}
		}
	}

	return nbr_counter;
}

void ofge_init(int node_index) {
	int i, j, k;
	char data_bytes[4];
	empty_rows = COL;
	num_solved = 0;
	num_no_update = 0;
	num_data = 0;
	// #if R_PCA
	// 	#if LOSSY_RPCA
			number_not_decoded = 0;
			number_not_decoded_2 = 0;
			number_not_decoded_3 = 0;
			actual_num_no_update = 0;
			actual_num_solved = 0;
			for(i = 0; i < NUM_RECORDS; i++){
				reliability[i] = 100;
				reliability_solved[i] = 100;
				reliability_time[i] = 0;
			}
			curr_elapsed = 0;
			// for(i = 0; i < 100; i++)
			// 	num_solved_log[i] = 1000;
	// 	#endif
	// #endif
	for(i=0;i<COL;i++) {
		solved[i] = 0;
		num_ones[i] = 0;
		pkt_decoded[i] = 0;
		// #if R_PCA
			no_update[i] = 0;
			update_control_nodes[i] = 0;
			// #if LOSSY_RPCA
				data_control[i] = 0;
				data_control_nodes[i] = 0;
		// 	#endif
		// #endif
		for(j=0;j<MERGE_LEN;j++) {
			G[i][j] = 0;
		}


		// ebramkw
		// initialize data Matrix
		for(j=0;j<PAYLOAD_LEN;j++) {
			Y[i][j] = 0;
		}
		
		// no more phases
		// if(phase != 2 && phase != 3){
		// 	for(j=0;j<PAYLOAD_LEN;j++) {
		// 		Y[i][j] = 0;
		// 	}
		// }
	}
	// #if TOTAL_NODES_RUN
	// 	for(i = CHAOS_NODES; i < TOTAL_NODES; i++){
	// 		#if R_PCA
	// 			update_control_nodes[i] = 0;
	// 			no_update[i] = 0;
	// 			#if LOSSY_RPCA
	// 				data_control[i] = 0;
	// 				data_control_nodes[i] = 0;
	// 			#endif
	// 		#endif
	// 	}
	// #endif
	// Initialise the following only if I am having data
	// Other relay nodes dont have node_index initialised
	if(has_data) {
		if(has_data_update){
			empty_rows--;
			solved[num_solved++] = node_index;
			num_ones[node_index] = 1;
			pkt_decoded[node_index] = 1;
			G[node_index][node_index/8] |= (1 << (node_index%8));

			data_control[node_index] = 1;
			data_control_nodes[num_data++] = node_index;

			// insert eigenvalue
            float2Bytes(singular_value, data_bytes);
            for(j = 0; j < 4; j++)
                 Y[node_index][j] = data_bytes[j];
            // insert corresponding eigenvector
            for(k = 1; k < (N + 1); k++){
                 float2Bytes(singular_vector[k - 1], data_bytes);
                 for(j = 0; j < 4; j++)
                         Y[node_index][(((k - 1) * 4)+(j + 4))] = data_bytes[j];
            }

            // no_update[node_index] = 0;
            // num_no_update = 0;
		} else{
			no_update[node_index] = 1;
			update_control_nodes[num_no_update++] = node_index;

			// data_control[node_index] = 0;
			// num_data = 0;
		}
	}

	reliability[0] = num_no_update + num_data;
	reliability_solved[0] = num_solved;

	#if ONE_SRC
	if(num_solved == 1)
		one_total_secs = 100;
	else
		one_total_secs = 0;
	#endif

	nbr_solved = 0;
	#if R_PCA
		nbr_updated = 0;
		#if LOSSY_RPCA
			nbr_data = 0;
			only_once = 0;
			prev_not_decoded = 0;
			prev_not_decoded_2 = 0;
			prev_not_decoded_3 = 0;
		#endif
	#endif
	nbr_count = 0;
	nbr_list_full = 0;
	//memset(nbr_solved_list,0,sizeof(nbr_solved_list));
	for(i=0;i<NBR_LIST;i++) {
		nbr_solved_list[i] = 100;
	}

	memset(nbr_solved_bv_list, 0, NBR_LIST * MERGE_LEN * sizeof(uint8_t));


	// for(i = 0; i < MERGE_LEN; i++){
	// 	update_control_bv[i] = 0;
	// 	data_control_bv[i] = 0;
	// 	solved_bv[i] = 0;
	// }
	memset(update_control_bv, 0, MERGE_LEN * sizeof(uint8_t));
	memset(data_control_bv, 0, MERGE_LEN * sizeof(uint8_t));
	memset(solved_bv, 0, MERGE_LEN * sizeof(uint8_t));

	// To compute latency keep track of slot number
	curr_slot = 0;
	slots = 0;
	ticks = 0;

	// // Flush encode counter
	// //memset(encode_counter,0,sizeof(encode_counter));
	// for(i=0;i<COL;i++)
	// 	encode_counter[i] = 0;

	neighbor_complete = 0;
	im_complete = 0;
	decoded_new = 0;

	half_decoded_new = 0;

	control_total_secs = 0;
	
	// wasted_packets = 0;
	// decoded_wasted_packets = 0;
	// discarded_wasted_packets = 0;
	// num_not_decoded_packets = 0;
	// total_packets = 0;
	// memset(undecoded_stored, 0, UNDECODED_SIZE * sizeof(uint8_t));
	// memset(undecoded_bv, 0, UNDECODED_SIZE * MERGE_LEN * sizeof(uint8_t));
	// memset(undecoded_payload, 0, UNDECODED_SIZE * PAYLOAD_LEN * sizeof(char));
	// undecoded_stored_num = 0;

	// memset(decode_count_log, 0, UNDECODED_SIZE * sizeof(int));
	// decode_count_log_counter = 0;

	// Processing log counter
	// dp_count = 0;

	// Activity counter
	//memset(tx_activity,0,sizeof(tx_activity));
	//memset(rx_activity,0,sizeof(rx_activity));
	//memset(rxn_activity,0,sizeof(rxn_activity));
	// tx_activity_cnt = 0;
	// rx_activity_cnt = 0;
	// rxn_activity_cnt = 0;
}
