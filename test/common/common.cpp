#include "common.h"

unsigned _BitInt(512) str2biguint(std::string data) {
    unsigned _BitInt(512) res = 0;
    int length = data.length();
    long long unsigned int val[8];
    for (int i = 7; i >= 0; i--) {
        val[7-i] = stoull(data.substr(16*i, 16), NULL, 16);
    }
    for (int i = 7; i >= 0; i--) {
        // cout << hex << val[i] << dec << endl;
        res = (res << 64) | val[i];
    }
    return res;
}

payload_t get_input(std::ifstream &infile) {
    std::string indata;
    int    empty;
    bool   last;

    infile >> last >> empty >> indata;

    unsigned _BitInt(512) tmp = str2biguint(indata);

    payload_t res;
    res.data = tmp;
    res.empty = empty;
    res.last = last;

    return res;
}
