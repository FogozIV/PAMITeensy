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
#include <QNEthernet.h>
#define HOSTNAME "mainrobotTeensy"
#define STATIC_IP_ADDR_STRNG "192.168.1.113"
#define GATEWAY_IP_ADDR_STRING "192.168.1.1"
#define SUBNET_MASK_STRING "255.255.255.0"

enum CustomEthernetStatus{
    UNABLE_TO_GET_IP_ADDRESS, CABLE_NOT_CONNECTED, OK
};

using namespace qindesign::network;
inline CustomEthernetStatus setupEthernet() {
    IPAddress myIP;
    IPAddress myNetmask;
    IPAddress myGW;

    myIP.fromString(STATIC_IP_ADDR_STRNG);
    myGW.fromString(GATEWAY_IP_ADDR_STRING);
    myNetmask.fromString(SUBNET_MASK_STRING);
    qindesign::network::Ethernet.begin(myIP, myNetmask, myGW);
    qindesign::network::Ethernet.setHostname(HOSTNAME);
    qindesign::network::MDNS.begin(HOSTNAME);
    if (!Ethernet.waitForLocalIP(2000)) {

        if (!Ethernet.linkStatus()) {
            return CABLE_NOT_CONNECTED;
        }
        return UNABLE_TO_GET_IP_ADDRESS;

    }
    return OK;
}

#endif //NETWORK_UTILS_H
