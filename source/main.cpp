/**
*   crc8 is implemented using the 3ds public domain implementaion
*   this referenced code can be found here: https://3dbrew.org/wiki/CRC-8-CCITT
*/

//Minimal imports for assessment 2
#include "mbed.h"
#include "LM75B.h"
#include "EthernetInterface.h"
#include "rtos.h"

/**
*   Variable below is part of the crc code
*   https://3dbrew.org/wiki/CRC-8-CCITT
*/
static const uint8_t CRC_TABLE[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

// variables
LM75B tmp(D14,D15);
InterruptIn sw2(SW2);
InterruptIn sw3(SW3);
InterruptIn joystick_up(A2);
InterruptIn joystick_down(A3);
InterruptIn joystick_left(A4);
InterruptIn joystick_right(A5);
InterruptIn joystick_fire(D4);

Thread ack_thread;

EthernetInterface network_interface;

uint16_t userid = 0xD281;
uint8_t seq_num = 0;
uint8_t packet[8];
uint8_t buffer = 0;
uint8_t packet_opts = 0;

const int ack_request_bit_flag = 1 << 0;
const int ccitt_bit_flag = 1 << 1;
const int retry_bit_flag = 1 << 3;

const int sw2_bit_flag = 1 << 0;
const int sw3_bit_flag = 1 << 1;
const int joystick_up_bit_flag = 1 << 2;
const int joystick_down_bit_flag = 1 << 3;
const int joystick_left_bit_flag = 1 << 4;
const int joystick_right_bit_flag = 1 << 5;
const int joystick_fire_bit_flag = 1 << 6;

// FLAGS
volatile int sw2_pressed_flag = 0;
volatile int sw3_pressed_flag = 0;
volatile int joystick_pressed_up_flag =  0;
volatile int joystick_pressed_down_flag = 0;
volatile int joystick_pressed_left_flag = 0;
volatile int joystick_pressed_right_flag = 0;
volatile int joystick_pressed_in_flag = 0;
volatile int ack_request_flag = 0;

void sw2_pressed()
{
    sw2_pressed_flag = 1;
}

void sw3_pressed()
{
    sw3_pressed_flag = 1;
}

void joystick_pressed_up()
{
    joystick_pressed_up_flag = 1;
}

void joystick_pressed_down()
{
    joystick_pressed_down_flag = 1;
}

void joystick_pressed_left()
{
    joystick_pressed_left_flag = 1;
}

void joystick_pressed_right()
{
    joystick_pressed_right_flag = 1;
}

void joystick_pressed_in()
{
    joystick_pressed_in_flag = 1;
}

/**
*   Resets all the flags and buffers to 0 ready for the next packet
*/
void reset_buffer()
{
    sw2_pressed_flag = 0;
    sw3_pressed_flag = 0;
    joystick_pressed_up_flag =  0;
    joystick_pressed_down_flag = 0;
    joystick_pressed_left_flag = 0;
    joystick_pressed_right_flag = 0;
    joystick_pressed_in_flag = 0;
    ack_request_flag = 0;
    buffer = 0;
    packet_opts = 0;
    packet_opts |= ccitt_bit_flag;
}

/**
*   From here the code is from the referenced source:
*   https://3dbrew.org/wiki/CRC-8-CCITT
*/
uint8_t crc8ccitt(const void * data, size_t size)
{
    uint8_t val = 0;

    uint8_t * pos = (uint8_t *) data;
    uint8_t * end = pos + size;

    while (pos < end) {
        val = CRC_TABLE[val ^ *pos];
        pos++;
    }

    return val;
}
/**
*   Referenced code ends here
*/

/**
*   returns the entered n value if the ack packet is what it should be
*   otherwise we want to retry so we set n to < 0
*/
int verify_ack(char recv_buffer[256], int n)
{
    bool verified = false;
    if(recv_buffer[0] != userid >> 8 || recv_buffer[1] != userid || recv_buffer[2] != seq_num) {
        return -1;
    } else {
        return n;
    }
}

/**
*   Generates the packet based on flags 
*   Sends the packet
*   and if necessary waits for an ack from the server
*/
void generate_packet()
{
    network_interface.connect();
    UDPSocket sock(&network_interface);
    SocketAddress sock_address;
    packet[0] = userid >> 8;
    packet[1] = userid;
    packet[2] = seq_num++;
    if(ack_request_flag) {
        packet_opts |= ack_request_bit_flag;
    }
    packet[3] = packet_opts;
    packet[4] = tmp.read();
    packet[5] = tmp.read() >> 8;
    if(sw2_pressed_flag) {
        buffer |= sw2_bit_flag;
    }
    if(sw3_pressed_flag) {
        buffer |= sw3_bit_flag;
    }
    if(joystick_pressed_up_flag) {
        buffer |= joystick_up_bit_flag;
    }
    if(joystick_pressed_down_flag) {
        buffer |= joystick_down_bit_flag;
    }
    if(joystick_pressed_left_flag) {
        buffer |= joystick_left_bit_flag;
    }
    if(joystick_pressed_right_flag) {
        buffer |= joystick_right_bit_flag;
    }
    if(joystick_pressed_in_flag) {
        buffer |= joystick_fire_bit_flag;
    }
    packet[6] = buffer;
    packet[7] = crc8ccitt(packet, 7);
    sock.sendto("lora.kent.ac.uk", 1789, packet, sizeof(packet));

    if(ack_request_flag) { // if we're expecting an ack
        char recv_buffer[256];
        int n = sock.recvfrom(&sock_address, recv_buffer, sizeof(recv_buffer));
        //n = verify_ack(recv_buffer, n);
        int wait_time = 60;
        packet_opts |= retry_bit_flag;
        packet[3] = packet_opts;
        while(n < 0) { // n < 0 means we're recv-ing a sever error
            wait(wait_time);
            sock.sendto("lora.kent.ac.uk", 1789, packet, sizeof(packet));
            n = sock.recvfrom(&sock_address, recv_buffer, sizeof(recv_buffer));
            //n = verify_ack(recv_buffer, n);
            wait_time += 60; // wait an extra minute
        }
    }
    reset_buffer();
}

/**
*   a thread that sets the ack_request_flag to 1 every minute
*/ 
void set_ack_request()
{
    while(1) {
        ack_request_flag = 1;
        if(seq_num == 0) { wait(10); }
        Thread::wait(60000);
    }
}

int main()
{
    ack_thread.start(set_ack_request);
    packet_opts |= ccitt_bit_flag;

    sw2.fall(&sw2_pressed);
    sw3.fall(&sw3_pressed);
    joystick_up.fall(&joystick_pressed_up);
    joystick_down.fall(&joystick_pressed_down);
    joystick_left.fall(&joystick_pressed_left);
    joystick_right.fall(&joystick_pressed_right);
    joystick_fire.fall(&joystick_pressed_in);

    while(1) {
        generate_packet();
        wait(10);
        //sleep();
    }
}
