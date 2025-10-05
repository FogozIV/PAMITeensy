//
// Created by fogoz on 02/05/2025.
//

#ifndef NETWORK_UTILS_H
#define NETWORK_UTILS_H

#ifndef htonll
#define htonll(x) ((1==htonl(1)) ? (x) : ((uint64_t)htonl((x) & 0xFFFFFFFF) << 32) | htonl((x) >> 32))
#endif
#ifndef ntohll
#define ntohll(x) ((1==ntohl(1)) ? (x) : ((uint64_t)ntohl((x) & 0xFFFFFFFF) << 32) | ntohl((x) >> 32))
#endif
#ifdef TEENSY41
#include <QNEthernet.h>
#endif
#ifndef HOSTNAME
#define HOSTNAME "mainrobotTeensy"
#endif
#ifdef STATIC_IP
#define STATIC_IP_ADDR_STRNG "192.168.1.113"
#define GATEWAY_IP_ADDR_STRING "192.168.1.1"
#define SUBNET_MASK_STRING "255.255.255.0"
#endif
enum CustomEthernetStatus{
    UNABLE_TO_GET_IP_ADDRESS, CABLE_NOT_CONNECTED, OK
};

#ifdef TEENSY41
using namespace qindesign::network;
#endif
CustomEthernetStatus setupEthernet();

#endif //NETWORK_UTILS_H
