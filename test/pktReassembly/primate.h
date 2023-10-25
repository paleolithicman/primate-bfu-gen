#ifndef _PRIMATE_
#define _PRIMATE_

#include "../common/common.h"

#define NUM_THREADS 16
#define NUM_THREADS_LG 4
#define REG_WIDTH 272
#define NUM_REGS_LG 5
#define OPCODE_WIDTH 6
#define IP_WIDTH 32

#define TCP_FIN 0
#define TCP_SYN 1
#define TCP_RST 2
#define TCP_FACK 4
#define PROT_UDP 0x11
#define PKT_FORWARD 0
#define PKT_DROP 1
#define PKT_CHECK 2
#define INSERT 1
#define UPDATE 2
#define DELETE 3

typedef unsigned _BitInt(96) flow_key_t;

struct tuple_t{
    uint16_t sPort;
    uint16_t dPort;
    uint32_t sIP;
    uint32_t dIP;

    inline void set(unsigned _BitInt(96) bv) {
        sPort = bv;
        dPort = bv >> 16;
        sIP = bv >> 32;
        dIP = bv >> 64;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = (((_BitInt(REG_WIDTH))dIP << 64) | ((_BitInt(REG_WIDTH))sIP << 32) | 
            ((_BitInt(REG_WIDTH))dPort << 16) | (_BitInt(REG_WIDTH))sPort);
        return val;
    }
};

struct meta_t{
    uint8_t prot;
    unsigned _BitInt(96) tuple;
    uint32_t seq;
    uint16_t len;
    unsigned _BitInt(30) hdr_len_flits_empty_pktID;
    unsigned _BitInt(9) tcp_flags;
    unsigned _BitInt(3) pkt_flags;
    unsigned _BitInt(58) last_7_bytes_pdu_flag;

    inline void set(unsigned _BitInt(252) bv) {
        prot = bv;
        tuple = bv >> 8;
        seq = bv >> 104;
        len = bv >> 136;
        hdr_len_flits_empty_pktID = bv >> 152;
        tcp_flags = bv >> 182;
        pkt_flags = bv >> 191;
        last_7_bytes_pdu_flag = bv >> 194;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = (((_BitInt(REG_WIDTH))last_7_bytes_pdu_flag << 194) | ((_BitInt(REG_WIDTH))pkt_flags << 191) |
            ((_BitInt(REG_WIDTH))tcp_flags << 182) | ((_BitInt(REG_WIDTH))hdr_len_flits_empty_pktID << 152) | ((_BitInt(REG_WIDTH))len << 136) |
            ((_BitInt(REG_WIDTH))seq << 104) | ((_BitInt(REG_WIDTH))tuple << 8) | (_BitInt(REG_WIDTH))prot);
        return val;
    }
};


struct fce_t{
    unsigned _BitInt(96) tuple;
    unsigned _BitInt(32) seq;
    unsigned _BitInt(10) pointer;
    unsigned _BitInt(10) slow_cnt;
    unsigned _BitInt(104) addr3_addr2_addr1_addr0_last_7_bytes;
    unsigned _BitInt(9) pointer2;
    unsigned _BitInt(5) ch0_bit_map;

    inline void set(unsigned _BitInt(266) bv) {
        tuple = bv;
        seq = bv >> 96;
        pointer = bv >> 128;
        slow_cnt = bv >> 138;
        addr3_addr2_addr1_addr0_last_7_bytes = bv >> 148;
        pointer2 = bv >> 252;
        ch0_bit_map = bv >> 261;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = (((_BitInt(REG_WIDTH))ch0_bit_map << 261) | ((_BitInt(REG_WIDTH))pointer2 << 252) |
            ((_BitInt(REG_WIDTH))addr3_addr2_addr1_addr0_last_7_bytes << 148) | ((_BitInt(REG_WIDTH))slow_cnt << 138) | ((_BitInt(REG_WIDTH))pointer << 128) |
            ((_BitInt(REG_WIDTH))seq << 96) | (_BitInt(REG_WIDTH))tuple);
        return val;
    }
};

struct dymem_t{
    // uint8_t prot;
    // _BitInt(96) tuple;
    // uint32_t seq;
    // uint16_t len;
    // _BitInt(30) hdr_len_flits_empty_pktID;
    // _BitInt(9) tcp_flags;
    // _BitInt(3) pkt_flags;
    // _BitInt(58) last_7_bytes_pdu_flag;
    meta_t meta;
    unsigned _BitInt(9) next;

    inline void set(unsigned _BitInt(261) bv) {
        meta.prot = bv;
        meta.tuple = bv >> 8;
        meta.seq = bv >> 104;
        meta.len = bv >> 136;
        meta.hdr_len_flits_empty_pktID = bv >> 152;
        meta.tcp_flags = bv >> 182;
        meta.pkt_flags = bv >> 191;
        meta.last_7_bytes_pdu_flag = bv >> 194;
        next = bv >> 252;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = (((_BitInt(REG_WIDTH))next << 252) | ((_BitInt(REG_WIDTH))meta.last_7_bytes_pdu_flag << 194) | 
            ((_BitInt(REG_WIDTH))meta.pkt_flags << 191) | ((_BitInt(REG_WIDTH))meta.tcp_flags << 182) | 
            ((_BitInt(REG_WIDTH))meta.hdr_len_flits_empty_pktID << 152) | ((_BitInt(REG_WIDTH))meta.len << 136) |
            ((_BitInt(REG_WIDTH))meta.seq << 104) | ((_BitInt(REG_WIDTH))meta.tuple << 8) | (_BitInt(REG_WIDTH))meta.prot);
        return val;
    }
};

void primate_main(primate_io &top_intf);

#endif