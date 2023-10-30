#ifndef _PRIMATE_
#define _PRIMATE_

#include "../common/common.h"

#define NUM_THREADS 16
#define NUM_THREADS_LG 4
#define REG_WIDTH 192
#define NUM_REGS_LG 5
#define OPCODE_WIDTH 6
#define IP_WIDTH 32

typedef unsigned _BitInt(48) macAddr_t;

typedef struct {
    unsigned _BitInt(48) dstAddr;
    unsigned _BitInt(48) srcAddr;
    unsigned _BitInt(16) etherType;

    inline void set(unsigned _BitInt(112) bv) {
        dstAddr = bv;
        srcAddr = (bv >> 48);
        etherType = (bv >> 96);
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = (((_BitInt(REG_WIDTH))etherType << 96) | ((_BitInt(REG_WIDTH))srcAddr << 48) | (_BitInt(REG_WIDTH))dstAddr);
        return val;
    }
} ethernet_t;

typedef struct {
    unsigned _BitInt(40) transportSpecific_domainNumber;
    unsigned _BitInt(8) reserved2;
    unsigned _BitInt(112) flags_reserved3;

    inline void set(unsigned _BitInt(160) bv) {
        transportSpecific_domainNumber = bv & 0xffffffffff;
        reserved2 = (bv >> 40) & 0xff;
        // flags_reserved3 = (bv >> 48) & 0xffffffffffffffffffffffffffff;
        flags_reserved3 = bv >> 48;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = (((_BitInt(REG_WIDTH))flags_reserved3 << 48) | ((_BitInt(REG_WIDTH))reserved2 << 40) | (_BitInt(REG_WIDTH))transportSpecific_domainNumber);
        return val;
    }
} ptp_l_t;

typedef struct {
    unsigned _BitInt(192) data;

    inline void set(unsigned _BitInt(192) bv) {
        data = bv;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        return data;
    }
} ptp_h_t;

typedef struct {
    uint16_t field_0;
    uint16_t field_1;
    uint16_t field_2;
    uint16_t field_3;

    inline void set(uint64_t bv) {
        field_0 = bv & 0xffff;
        field_1 = (bv >> 16) & 0xffff;
        field_2 = (bv >> 32) & 0xffff;
        field_3 = (bv >> 48) & 0xffff;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = ((_BitInt(REG_WIDTH))field_3 << 48) | ((_BitInt(REG_WIDTH))field_2 << 32) | ((_BitInt(REG_WIDTH))field_1 << 16) | (_BitInt(REG_WIDTH))field_0;
        return val;
    }
} header_t;

typedef struct {
    header_t hdr0;
    header_t hdr1;

    inline void set(unsigned _BitInt(128) bv) {
        hdr0.set(bv);
        hdr1.set(bv >> 64);
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(64) hdr0_uint = hdr0.to_uint();
        unsigned _BitInt(64) hdr1_uint = hdr1.to_uint();
        unsigned _BitInt(REG_WIDTH) val = ((_BitInt(REG_WIDTH))hdr1_uint << 64) | (_BitInt(REG_WIDTH))hdr0_uint;
        return val;
    }
} header2_t;

typedef struct {
    unsigned _BitInt(72) version_ttl;
    unsigned _BitInt(8) protocol;
    unsigned _BitInt(80) hdrChecksum_dstAddr;

    inline void set(unsigned _BitInt(160) bv) {
        version_ttl = bv;
        protocol = (bv >> 72) & 0xff;
        hdrChecksum_dstAddr = bv >> 80;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = ((_BitInt(REG_WIDTH))hdrChecksum_dstAddr << 80) | ((_BitInt(REG_WIDTH))protocol << 72) | (_BitInt(REG_WIDTH))version_ttl;
        return val;
    }
} ipv4_t;

typedef struct {
    unsigned _BitInt(160) srcPort_urgentPtr;

    inline void set(unsigned _BitInt(160) bv) {
        srcPort_urgentPtr = bv;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        return srcPort_urgentPtr;
    }
} tcp_t;

typedef struct {
    unsigned _BitInt(64) srcPort_checksum;

    inline void set(unsigned _BitInt(64) bv) {
        srcPort_checksum = bv;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        return srcPort_checksum;
    }
} udp_t;

typedef struct {
    uint16_t egress_spec;
    uint16_t mcast_grp;

    inline void set(uint32_t bv) {
        egress_spec = bv & 0xffff;
        mcast_grp = bv >> 16;
    }

    inline unsigned _BitInt(REG_WIDTH) to_uint() {
        unsigned _BitInt(REG_WIDTH) val = ((_BitInt(REG_WIDTH))mcast_grp << 16) | (_BitInt(REG_WIDTH))egress_spec;
        return val;
    }
} standard_metadata_t;

void primate_main(primate_io &top_intf);

// inline unsigned _BitInt(REG_WIDTH> cat_header(header_t hdr1, header_t hdr0) {
//     unsigned _BitInt(REG_WIDTH> val = (0, hdr1.field_3, hdr1.field_2, hdr1.field_1, hdr1.field_0, hdr0.field_3, hdr0.field_2, hdr0.field_1, hdr0.field_0);
//     return val;
// }

#endif
