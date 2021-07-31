/**
 * Authored by Victor Freire (https://xu.me.wisc.edu/), June 2021
 *
 *
 *
 */

#ifndef _AIDECK_H_
#define _AIDECK_H_

#include "stabilizer_types.h"
#include "controller_lqr.h"

#ifdef CBF_TYPE_POS
#define MAX_CBFPACKET_DATA_SIZE 20
#elif CBF_TYPE_EUL
#ifdef CBF_ITERS
#define MAX_CBFPACKET_DATA_SIZE 20
#else
#define MAX_CBFPACKET_DATA_SIZE 16
#endif // CBF_ITERS
#else
#define MAX_CBFPACKET_DATA_SIZE 0
#endif

/**
 * UART Packet structure (MAX_CBF_PACKET_DATA_SIZE+1)Bytes
 * Header:
 * 'V' = healthy packet
 * 'X' = stop packet
 */
typedef union{
  struct{
    uint8_t header;
    uint8_t data[MAX_CBFPACKET_DATA_SIZE];
  };
  uint8_t raw[MAX_CBFPACKET_DATA_SIZE+1];
} __attribute__((packed)) CBFPacket;


#ifdef CBF_TYPE_POS
/** Control Input to adjust in the CBF-QP 16Bytes */
typedef struct u_s{
  float T;       // Normalized thrust [m/s^2]
  float phi;     // Roll [rad]
  float theta;   // Pitch [rad]
  float psi;     // Yaw [rad]
} __attribute__((packed)) u_t;

/** Compressed version of u_s 8Bytes*/
typedef struct u_comp_s{
  int16_t T;     // Normalized thrust [mm/s^2]
  int16_t phi;   // Roll [milirad]
  int16_t theta; // Pitch [milirad]
  int16_t psi;   // Yaw [milirad]
} __attribute__((packed)) u_comp_t;

/** OSQP Parametric data to be updated in the OSQPData struct 40Bytes */
typedef struct cbf_qpdata_s{
  float x;      // Position [m]
  float y;
  float z;
  float x_dot;  // Velocity [m/s]
  float y_dot;
  float z_dot;
  u_t u;        // Nominal input to adjust
} cbf_qpdata_t;

/** Compressed version of cbf_qpdata_s 20Bytes */
typedef struct cbf_qpdata_comp_s{
  int16_t x;     // Position [mm]
  int16_t y;
  int16_t z;
  int16_t x_dot; // Velocity [mm/s]
  int16_t y_dot;
  int16_t z_dot;
  u_comp_t u;    // Compressed Nominal input to adjust
} __attribute__((packed)) cbf_qpdata_comp_t;

#elif CBF_TYPE_EUL
/** Control Input to adjust in the CBF-QP 16Bytes */
typedef struct u_s{
  float T;  // Normalized thrust [m/s^2]
  float p;  // Angular velocities [rad/s]
  float q;
  float r;
} __attribute__((packed)) u_t;

/** Compressed version of u_s 8Bytes*/
typedef struct u_comp_s{
  int16_t T;  // Normalized thrust [mm/s^2]
  int16_t p;  // Angular velocities [milirad/s]
  int16_t q;
  int16_t r;
} __attribute__((packed)) u_comp_t;


/** OSQP Parametric data to be updated in the OSQPData struct 24Bytes */
typedef struct cbf_qpdata_s{
  float phi;   // Roll  [rad]
  float theta; // Pitch [rad]
  u_t u;       // Nominal input to adjust
} cbf_qpdata_t;

/** Compressed version of cbf_qpdata_eul_s 12Bytes */
typedef struct cbf_qpdata_comp_s{
  int16_t phi;   // Roll  [milirad]
  int16_t theta; // Pitch [milirad]
  u_comp_t u;     // Compressed Nominal input to adjust
} __attribute__((packed)) cbf_qpdata_comp_t;
#endif // CBF_TYPE

#ifdef CBF_ITERS
/** Extended structure to send iters sizeof(u_t)+2Bytes */
typedef struct u_it_s{
  u_t u;
  uint16_t iters; // Number of iterations to solve CBF-QP [-]
} __attribute__((packed)) u_it_t;
#endif

/**
 * Pack size bytes of data into a CBFPacket
 *
 * returns a pointer to the CBFPacket if successful
 *         NULL if size of data is too large
 */
CBFPacket *cbf_pack(const uint8_t size, uint8_t *data);


#if defined CBF_TYPE_EUL || defined CBF_TYPE_POS
/**
 * Send the CBF-QP parametric data to the AI Deck to update and solve the problem
 */
void aideck_send_cbf_data(const cbf_qpdata_t *data);
#endif

/**
 * Update controller variables with most recently received
 * solution to CBF-QP
 */
void aideck_get_safe_u(float *u);


#endif /* _AIDECK_H_ */


