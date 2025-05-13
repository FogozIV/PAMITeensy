//
// Created by fogoz on 13/05/2025.
//

#include "utils/NetworkUtils.h"


CustomEthernetStatus FLASHMEM setupEthernet() {
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