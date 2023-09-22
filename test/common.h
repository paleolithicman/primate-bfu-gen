#ifndef _COMMON_
#define _COMMON_

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>

#define IO_W 512
#define IO_BW 64
#define IO_BW_LG 6

struct payload_t {
    unsigned _BitInt(IO_W) data;
    int empty;
    bool last;

    inline friend std::ostream& operator<<(std::ostream& os, const payload_t& val) {
        long long unsigned int array[IO_W/64];
        _BitInt(IO_W) tmp = val.data;
        for (int i = 0; i < IO_W/64; i++) {
            array[i] = tmp;
            tmp = tmp >> 64;
        }
        os << std::hex << "data = ";
        for (int i = IO_W/64-1; i >= 0; i--) {
            // os << std::setw(16) << std::setfill('0') << array[i];
            os << array[i];
        }
        os << std::dec << "; empty = " << val.empty << "; last = " << val.last << std::endl;
        return os;
    }
};

unsigned _BitInt(512) str2biguint(std::string data);

payload_t get_input(std::ifstream &infile);

class primate_io {
    unsigned _BitInt(IO_W) input_buf;
    int input_buf_len;
    int fifo_empty;
    bool last_buf;
    payload_t payload;
    bool payload_v;

    std::vector<payload_t> pkt_buf;

    std::ifstream infile;
    std::ofstream outfile;

public:

    primate_io() {
        input_buf_len = 0;
        fifo_empty = 0;
        last_buf = false;
        pkt_buf.clear();
    }

    void Init() {
        input_buf_len = 0;
        fifo_empty = 0;
        last_buf = false;
        payload_v = false;
        pkt_buf.clear();
        infile.open("input.txt");
        outfile.open("output.txt");
    }

    template<typename h_t>
    void Input_header(const int length, h_t &header) {
        bool res_valid = false;
        int shift;

        if (!payload_v) {
            payload = get_input(infile);
            payload_v = true;
        }

        while (!res_valid) {
            shift = 0;
            if (length <= input_buf_len) {
                header.set(input_buf);
                shift = length;
                res_valid = true;
            }

            // shift and fill input buffer
            // std::cout << "input_buf_len: " << input_buf_len << ", shift: " << shift << ", fifo_empty: " << fifo_empty << std::endl;
            if (input_buf_len == 0) {
                input_buf = payload.data;
            } else {
                input_buf = ((payload.data >> (fifo_empty*8)) << (input_buf_len*8 - shift*8)) | 
                    ((input_buf << (512 - input_buf_len*8)) >> (512 - input_buf_len*8 + shift*8));
            }

            if (input_buf_len - shift <= fifo_empty) {
                // fill input buffer and pop input
                input_buf_len = input_buf_len - shift + IO_BW - fifo_empty;
                last_buf = payload.last;
                fifo_empty = 0;
                payload = get_input(infile);
            } else {
                // fill input buffer
                fifo_empty += (IO_BW - input_buf_len + shift);
                input_buf_len = IO_BW;
            }
        }
    }

    void Input_done() {
        payload_t pl;
        pl.data = input_buf;
        pl.empty = IO_BW - input_buf_len + fifo_empty;
        pl.last = last_buf;
        pkt_buf.push_back(pl);
        input_buf_len = 0;
        fifo_empty = 0;
        if (!last_buf) {
            pkt_buf.push_back(payload);
            payload_v = false;
            bool is_last = payload.last;
            while (!is_last) {
                bool valid = false;
                payload = get_input(infile);
                is_last = payload.last;
                pkt_buf.push_back(payload);
            }
        }
    }

    template<typename h_t>
    void Output_header(const int length, h_t &header) {
        payload_t pl;
        pl.data = header.to_uint();
        pl.empty = IO_BW - length;
        pl.last = false;
        outfile << pl;
    }

    void Output_done() {
        for (auto it = pkt_buf.begin(); it != pkt_buf.end(); it++) {
            outfile << (*it);
        }
        pkt_buf.clear();
    }

    template<typename meta_t>
    void Output_meta(meta_t &standard_metadata) {
        std::cout << "port: " << standard_metadata.egress_spec << std::endl;
    } //outputMeta inst

};


#endif
