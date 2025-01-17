#ifndef _COMMON_
#define _COMMON_

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>

//#define IO_W 512
//#define IO_BW 64
//#define IO_BW_LG 6

#define IO_W 256
#define IO_BW 32
#define IO_BW_LG 5

struct payload_t {
    unsigned _BitInt(IO_W) data;
    int empty;
    bool last;

    inline friend std::ostream& operator<<(std::ostream& os, const payload_t& val) {
        long long unsigned int array[IO_W/64];
        unsigned _BitInt(IO_W) tmp = val.data;
        for (int i = 0; i < IO_W/64; i++) {
            array[i] = tmp;
            tmp = tmp >> 64;
        }
        os << std::hex << "data = ";
        for (int i = IO_W/64-1; i >= 0; i--) {
            os << std::setw(16) << std::setfill('0') << array[i];
            // os << array[i];
        }
        os << std::dec << "; empty = " << val.empty << "; last = " << val.last << std::endl;
        return os;
    }
};

std::ostream& operator<<(std::ostream& os, const unsigned _BitInt(IO_W) &val);

unsigned _BitInt(IO_W) str2biguint(std::string data);

payload_t get_input(std::ifstream &infile);

class primate_io {
    unsigned _BitInt(IO_W) input_buf;
    unsigned pktID;
    unsigned flits;
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
        pktID = 0;
        flits = 0;
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
            // std::cout << "input_buf: " << input_buf << std::endl;
            if (input_buf_len == 0 || (input_buf_len == shift)) {
                input_buf = payload.data;
            } else {
                input_buf = ((payload.data >> (fifo_empty*8)) << (input_buf_len*8 - shift*8)) | 
                    ((input_buf << (IO_W - input_buf_len*8)) >> (IO_W - input_buf_len*8 + shift*8));
            }

            if (input_buf_len - shift <= fifo_empty) {
                // fill input buffer and pop input
                input_buf_len = input_buf_len - shift + IO_BW - fifo_empty;
                last_buf = payload.last;
                fifo_empty = 0;
                payload = get_input(infile);
                flits++;
            } else {
                // fill input buffer
                fifo_empty += (IO_BW - input_buf_len + shift);
                input_buf_len = IO_BW;
            }
        }
    }

    template<typename h0_t, typename h1_t>
    void Input_2header(const int length0, h0_t &header0, const int length1, h1_t &header1) {
        Input_header<h0_t>(length0, header0);
        Input_header<h1_t>(length1, header1);
    }

    template<typename h_t>
    void Input_simple(h_t &header) {
        Input_header<h_t>(IO_BW, header);
    }

    void Input_done() {
        if (fifo_empty != 0) {
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
        flits = 0;
        pktID++;
    }

    template<typename meta_t>
    void Input_done(meta_t &metadata) {
        payload_t pl;
        pl.data = input_buf;
        pl.empty = IO_BW - input_buf_len + fifo_empty;
        pl.last = last_buf;
        pkt_buf.push_back(pl);
        metadata.empty = IO_BW - input_buf_len + fifo_empty;
        metadata.pktID = pktID;
        input_buf_len = 0;
        fifo_empty = 0;
        pktID++;
        if (!last_buf) {
            pkt_buf.push_back(payload);
            payload_v = false;
            bool is_last = payload.last;
            while (!is_last) {
                bool valid = false;
                payload = get_input(infile);
                flits++;
                is_last = payload.last;
                pkt_buf.push_back(payload);
            }
        }
        metadata.flits = flits;
        flits = 0;
    }

    template<typename h_t>
    void Output_header(const int length, h_t &header) {
        payload_t pl;
        pl.data = header.to_uint();
        pl.empty = IO_BW - length;
        pl.last = false;
        if (pkt_buf.empty()) {
            pl.last = true;
        }
        outfile << pl;
    }

    template<typename h0_t, typename h1_t>
    void Output_2header(const int length0, h0_t &header0, const int length1, h1_t &header1) {
        Output_header<h0_t>(length0, header0);
        Output_header<h1_t>(length1, header1);
    }

    template<typename h_t>
    void Output_simple(h_t &header) {
        Output_header<h_t>(IO_BW, header);
    }

    void Output_done() {
        for (auto it = pkt_buf.begin(); it != pkt_buf.end(); it++) {
            outfile << (*it);
        }
        pkt_buf.clear();
    }

    template<typename meta_t>
    void Output_meta(meta_t &standard_metadata) {
        std::cout << standard_metadata.egress_spec << std::endl;
    } //outputMeta inst

};


#endif
