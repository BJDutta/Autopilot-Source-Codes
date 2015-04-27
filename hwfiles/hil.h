#ifndef MLINK_H
#define MLINK_H




void send_HIL();
void parse_HIL_data(unsigned char buff[], int len);
void calculate_hil();
void calculate_hil_gps();
void calculate_hil_imu();








#endif
