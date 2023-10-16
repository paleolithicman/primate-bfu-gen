#include "primate.h"
#include "bfs.h"

void primate_main(primate_io &top_intf) {
    int hdr_count;
    ethernet_t eth;
    ptp_l_t ptp_l;
    ptp_h_t ptp_h;
    ipv4_t ipv4;
    tcp_t tcp;
    udp_t udp;
    header_t header_0;
    header_t header_1;
    header_t header_2;
    header_t header_3;
    header_t header_4;
    header_t header_5;
    header_t header_6;
    header_t header_7;

    hdr_count = 0;
    top_intf.Input_header<ethernet_t>(14, eth);
    if (eth.etherType == 0x88f7) {
        top_intf.Input_header<ptp_l_t>(20, ptp_l);
        top_intf.Input_header<ptp_h_t>(24, ptp_h);
        hdr_count = 1;
        if (ptp_l.reserved2 == 1) {
            top_intf.Input_header<header_t>(8, header_0);
            hdr_count = 2;
            if (header_0.field_0 != 0) {
                top_intf.Input_header<header_t>(8, header_1);
                hdr_count = 3;
                if (header_1.field_0 != 0) {
                    top_intf.Input_header<header_t>(8, header_2);
                    hdr_count = 4;
                    if (header_2.field_0 != 0) {
                        top_intf.Input_header<header_t>(8, header_3);
                        hdr_count = 5;
                        if (header_3.field_0 != 0) {
                            top_intf.Input_header<header_t>(8, header_4);
                            hdr_count = 6;
                            if (header_4.field_0 != 0) {
                                top_intf.Input_header<header_t>(8, header_5);
                                hdr_count = 7;
                                if (header_5.field_0 != 0) {
                                    top_intf.Input_header<header_t>(8, header_6);
                                    hdr_count = 8;
                                    if (header_6.field_0 != 0) {
                                        top_intf.Input_header<header_t>(8, header_7);
                                        hdr_count = 9;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    } else if (eth.etherType == 0x800) {
        top_intf.Input_header<ipv4_t>(20, ipv4);
        hdr_count = 10;
        if (ipv4.protocol == 6) {
            top_intf.Input_header<tcp_t>(20, tcp);
            hdr_count = 11;
        } else if (ipv4.protocol == 0x11) {
            top_intf.Input_header<udp_t>(8, udp);
            hdr_count = 12;
        }
    }
    top_intf.Input_done();

    // Ingress
    int flag;
    standard_metadata_t standard_metadata;

    standard_metadata.egress_spec = 0;
    standard_metadata.mcast_grp = 0;
    uint16_t port;
    flag = forward_exact(eth.dstAddr, port);
    switch (flag) {
    case 0:
        standard_metadata.egress_spec = port;
        break;
    case 1:
        standard_metadata.egress_spec = 0x1ff;
        break;
    }

    top_intf.Output_meta<standard_metadata_t>(standard_metadata);
    top_intf.Output_header<ethernet_t>(14, eth);
    if (hdr_count < 10) {
        top_intf.Output_2header<ptp_l_t, ptp_h_t>(20, ptp_l, 24, ptp_h);
        if (hdr_count > 1) {
            if (hdr_count > 2) {
                top_intf.Output_2header<header_t, header_t>(8, header_0, 8, header_1);
                if (hdr_count > 3) {
                    if (hdr_count > 4) {
                        top_intf.Output_2header<header_t, header_t>(8, header_2, 8, header_3);
                        if (hdr_count > 5) {
                            if (hdr_count > 6) {
                                top_intf.Output_2header<header_t, header_t>(8, header_4, 8, header_5);
                                if (hdr_count > 7) {
                                    if (hdr_count > 8) {
                                        top_intf.Output_2header<header_t>(8, header_6, 8, header_7);
                                    } else {
                                        top_intf.Output_header<header_t>(8, header_6);
                                    }
                                }
                            } else {
                                top_intf.Output_header<header_t>(8, header_4);
                            }
                        }
                    } else {
                        top_intf.Output_header<header_t>(8, header_2);
                    }
                }
            } else {
                top_intf.Output_header<header_t>(8, header_0);
            }
        }
    } else {
        if (hdr_count == 11) {
            top_intf.Output_2header<ipv4_t, tcp_t>(20, ipv4, 20, tcp);
        } else {
            top_intf.Output_2header<ipv4_t, udp_t>(20, ipv4, 8, udp);
        }
    }
    top_intf.Output_done();
}
