//
// Created by fogoz on 02/05/2025.
//

#ifndef SETUPETHERNET_H
#define SETUPETHERNET_H
#include <QNEthernet.h>
#define HOSTNAME "MyTeensyWincyProj"
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
    IPAddress mydnsServer;

    qindesign::network::Ethernet.setHostname(HOSTNAME);
    myIP.fromString(STATIC_IP_ADDR_STRNG);
    myGW.fromString(GATEWAY_IP_ADDR_STRING);
    myNetmask.fromString(SUBNET_MASK_STRING);
    qindesign::network::Ethernet.begin(myIP, myNetmask, myGW);
    qindesign::network::Ethernet.setDNSServerIP(mydnsServer);
    if (!Ethernet.waitForLocalIP(2000)) {

        if (!Ethernet.linkStatus()) {
            return CABLE_NOT_CONNECTED;
        }
        return UNABLE_TO_GET_IP_ADDRESS;

    }
    return OK;
}

#endif //SETUPETHERNET_H
