#include "bfs.h"

int forward_exact(macAddr_t &dstAddr, uint16_t &port) {
    if (dstAddr == 0x0cc47aa32534) {
        port = 1;
        return 0;
    } else if (dstAddr == 0x0cc47aa32535) {
        port = 2;
        return 0;
    }
    return 1;
}
