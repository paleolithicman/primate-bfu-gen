#include "common.h"
#include <cmath>

unsigned _BitInt(IO_W) str2biguint(std::string data) {
    unsigned _BitInt(IO_W) res = 0;
    int length = std::ceil(IO_W/64.0);
    long long unsigned int val[length];
    for (int i = length-1; i >= 0; i--) {
        val[length-1-i] = stoull(data.substr(16*i, 16), NULL, 16);
    }
    for (int i = length-1; i >= 0; i--) {
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

    unsigned _BitInt(IO_W) tmp = str2biguint(indata);

    payload_t res;
    res.data = tmp;
    res.empty = empty;
    res.last = last;

    return res;
}

std::ostream& operator<<(std::ostream& os, const unsigned _BitInt(IO_W) &val) {
    long long unsigned int array[IO_W/64];
    unsigned _BitInt(IO_W) tmp = val;
    for (int i = 0; i < IO_W/64; i++) {
        array[i] = tmp;
        tmp = tmp >> 64;
    }
    os << std::hex;
    for (int i = IO_W/64-1; i >= 0; i--) {
        os << std::setw(16) << std::setfill('0') << array[i];
    }
    os << std::dec;
    return os;
}
