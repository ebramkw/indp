#include "indp-test.h"

#define RLNC 1

#define DEBUG 0
#define COL CHAOS_NODES

// ebramkw: check if pos is 1 in var bits
/* Usage: CHECK_BIT(temp, 3) */
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

/* Global Variables */
extern int solved[COL];
extern int num_solved;
extern int nbr_solved;
extern int nbr_source;
extern int nbr_destination;

/* Generate multiple BV packets */
#define NUM_PKTS 1
extern int decode_count;
extern int curr_degree;

// N: For fast termination
extern int neighbor_complete;
extern int im_complete;
extern int decoded_new;

// M: Neighbor info
extern uint8_t solved_bv[MERGE_LEN];

// ebramkw: channel hopping
#if CHANNEL_HOPPING
	#define CHANNELS_NUM 2
	extern int RF_CHANNELS[CHANNELS_NUM];
	extern int curr_rf_channel;
	extern int switched;
	// extern int recevied_num_per_channel[CHANNELS_NUM];
#endif

// ebramkw
// used for debugging
// extern uint8_t bv[MERGE_LEN];
// extern uint8_t payload_v[PAYLOAD_LEN];

// MM: Temporary global
extern int missing_pkt;
extern int missing_list[CHAOS_NODES];
extern int missing_list_ptr[CHAOS_NODES];
extern int missing_list_count;

// MM: Nbr solved list
#define NBR_LIST 5
extern uint8_t nbr_solved_bv_list[NBR_LIST][MERGE_LEN];
extern int nbr_solved_list[NBR_LIST];
extern int nbr_common_list[NBR_LIST];
extern int nbr_source_list[NBR_LIST];
extern int nbr_relay_list[NBR_LIST];
extern int nbr_count;

/* Function declarations */
// ebramkw: add code vector
void encode(uint8_t*, char*);				// Encoder for OFGE

// ebramkw: add data
#if R_PCA
	#if LOSSY_RPCA
		int decode(uint8_t*, uint8_t*, char*, uint8_t*, uint8_t*);
	#else
		int decode(uint8_t*, uint8_t*, char*, uint8_t*);		// Decoder for OFGE
	#endif
#else
	int decode(uint8_t*, uint8_t*, char*);		// Decoder for OFGE
#endif

// ebramkw: send y vector
void substitution(uint8_t, uint8_t*, char*);	// Performs backward/forward substition
int find_min(int*);								// Returns minimum element in array
int recv_degree();								// Return degree based on how much neighbors have solved	
uint8_t leftmost_one(uint8_t*);					// Returns index of leftmost one
uint8_t degree_eq(uint8_t*);					// Returns degree of given equation
void xor_bv_matrix(uint8_t*, uint8_t);			// XOR: bv = bv^row

// ebramkw
void xor_yv_matrix(char*, uint8_t);			// ebramkw: XOR: yv = yv^row


void xor_matrix_bv(uint8_t, uint8_t*);			// XOR: row = row^eq

// ebramkw
void xor_matrix_yv(uint8_t, char*);			// XOR: row = row^eq


void xor_matrix_matrix(uint8_t, uint8_t);		// XOR: row = row^row

// ebramkw
void xor_matrix_matrix_Y(uint8_t, uint8_t);		// XOR: row = row^row


void swap_eq(uint8_t*, uint8_t);				// SWAP: Two equations

// ebramkw
void swap_code(char*, uint8_t);				// SWAP: Two code, calculated and Matrix row


void copy_eq(uint8_t*, uint8_t);				// COPY: Matrix row to Equation

// ebramkw
void copy_code(char*, uint8_t);				// COPY: claculated code to Matrix row


void print_uint8_t(uint8_t);					// PRINT: Element of matrix
void print_g();									// PRINT: Matrix
void ofge_init();								// Initialize global variables

// ebramkw: convert float to array of bytes
void float2Bytes(float, char*);

// ebramkw
uint8_t no_neighbor_has_my_data();
extern int half_decoded_new;

extern uint8_t CHAOS_BOOTSTRAPPING;
extern unsigned short nodes[CHAOS_NODES];
// extern int total_packets;
// extern int wasted_packets;
// extern int num_not_decoded_packets;
// extern int decoded_wasted_packets;
// extern int discarded_wasted_packets;
// #define UNDECODED_SIZE 10
// extern uint8_t undecoded_stored[UNDECODED_SIZE];
// extern int undecoded_stored_num;
// extern uint8_t undecoded_bv[UNDECODED_SIZE][MERGE_LEN];
// extern char undecoded_payload[UNDECODED_SIZE][PAYLOAD_LEN];
// extern int decode_count_log[UNDECODED_SIZE];
// extern int decode_count_log_counter;

#if R_PCA
	#if TOTAL_NODES_RUN
		// uint8_t send_data(int);
		extern uint8_t no_update[TOTAL_NODES];
		extern uint8_t update_control_bv[TOTAL_MERGE_LEN];
		extern int update_control_nodes[TOTAL_NODES];
	#else
		extern uint8_t no_update[COL];
		extern uint8_t update_control_bv[MERGE_LEN];
		extern int update_control_nodes[COL];
	#endif
	extern int num_no_update;
	extern int nbr_updated;
	#if LOSSY_RPCA
		#if TOTAL_NODES_RUN
			extern uint8_t data_control[TOTAL_NODES];
			extern uint8_t data_control_bv[TOTAL_MERGE_LEN];
			extern int data_control_nodes[TOTAL_NODES];
		#else
			extern uint8_t data_control[COL];
			extern uint8_t data_control_bv[MERGE_LEN];
			extern int data_control_nodes[COL];
		#endif
		extern int num_data;
		extern int nbr_data;
		extern int number_not_decoded;
		extern int number_not_decoded_2;
		extern int number_not_decoded_3;
		extern uint8_t only_once;
		extern uint8_t prev_not_decoded;
		extern int update_decoded_new;
		#define NUM_RECORDS 16
		extern unsigned long target_elapsed[NUM_RECORDS];
		extern uint8_t reliability[NUM_RECORDS];
		extern uint8_t reliability_solved[NUM_RECORDS];
		extern unsigned long reliability_time[NUM_RECORDS];
		extern unsigned long curr_elapsed;
		// extern unsigned long num_solved_log[100];
	#endif
#endif

// TX Condition
extern int tx_cond[2][2][2];

// Slot Activity:
#define ACTIVITY_LOG_SIZE 200
extern int curr_relay;
extern int tx_activity[ACTIVITY_LOG_SIZE];
extern int tx_activity_cnt;
extern int rx_activity[ACTIVITY_LOG_SIZE];
extern int rx_activity_cnt;
extern int rxn_activity[ACTIVITY_LOG_SIZE];
extern int rxn_activity_cnt;

// Log Encoding/Decoding Time
extern rtimer_clock_t encode_time_log[ACTIVITY_LOG_SIZE];
extern rtimer_clock_t decode_time_log[ACTIVITY_LOG_SIZE];
extern rtimer_clock_t encode_time;
extern rtimer_clock_t decode_time;
extern int dp_count;

// Log help info
extern int tx_to_nbr[ACTIVITY_LOG_SIZE][NBR_LIST];
extern int rx_from_nbr[ACTIVITY_LOG_SIZE][NBR_LIST];
extern int rxn_from_nbr[ACTIVITY_LOG_SIZE][NBR_LIST];

/** @} */