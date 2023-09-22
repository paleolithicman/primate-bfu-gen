#include "primate.h"

int main() {
    primate_io top_intf;
    top_intf.Init();
    int NUM_PKT = 512;

    for (int i = 0; i < NUM_PKT; i++) {
        primate_main(top_intf);
    }

    return 0;
}
