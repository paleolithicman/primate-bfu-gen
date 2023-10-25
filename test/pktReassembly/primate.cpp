#include "primate.h"
#include "bfs.h"

void primate_main(primate_io &top_intf) {
    fce_t fte;
    meta_t input;
    top_intf.Input_simple<meta_t>(input);
    if ((input.tcp_flags == (1 << TCP_FACK)) && (input.len == 0)) {
        // std::cout << "ack packet\n";
        input.pkt_flags = PKT_FORWARD;
        top_intf.Output_simple<meta_t>(input);
        return;
    } else if (input.prot == PROT_UDP) {
        // std::cout << "udp packet\n";
        input.pkt_flags = PKT_CHECK;
        top_intf.Output_simple<meta_t>(input);
        return;
    } else {
        if (input.len != 0) input.pkt_flags = PKT_CHECK;
        else input.pkt_flags = PKT_FORWARD;
        // std::cout << "tcp packet, seq: " << (unsigned)input.seq << ", length: " << input.len << ", flags: " << (unsigned)input.tcp_flags << "\n";
        primate_lock(&(input.tuple));
        flow_table_read(&input, &fte);
        if (fte.ch0_bit_map != 0) { // ch0_bit_map shows which sub-table it hits
            // Flow exists
            // std::cout << "flow exists, seq: " << (unsigned)fte.seq << "\n";
            if (input.seq == fte.seq) {
                if (fte.slow_cnt > 0) {
                    dymem_t pkt;
                    dymem_t pkt_next;
                    pkt.meta = input;
                    do {
                        dymem_lookup((unsigned _BitInt(9)*)&(fte.pointer), &pkt_next);
                        if (pkt.meta.seq + pkt.meta.len == pkt_next.meta.seq) {
                            dymem_free((unsigned _BitInt(9)*)&(fte.pointer));
                            top_intf.Output_simple<meta_t>(pkt.meta);
                            fte.pointer = pkt_next.next;
                            pkt = pkt_next;
                            fte.slow_cnt--;
                        } else {
                            break;
                        }
                    } while (fte.slow_cnt > 0);
                    // update FT
                    if ((input.tcp_flags & (1 << TCP_FIN) | (input.tcp_flags & (1 << TCP_RST))) != 0) {
                        flow_table_delete(&fte);
                    } else {
                        fte.seq = pkt.meta.seq + pkt.meta.len;
                        flow_table_update(&fte);
                    }
                    primate_unlock(&(input.tuple));
                    top_intf.Output_simple<meta_t>(pkt.meta);
                    return;
                } else {
                    // in order packet
                    // std::cout << std::hex << "tcp flags: " << (unsigned)input.tcp_flags << std::dec << "\n";
                    if ((input.tcp_flags & (1 << TCP_FIN) | (input.tcp_flags & (1 << TCP_RST))) != 0) {
                        flow_table_delete(&fte);
                    } else {
                        fte.seq = input.seq + input.len;
                        flow_table_update(&fte);
                    }
                    primate_unlock(&(input.tuple));
                    top_intf.Output_simple<meta_t>(input);
                    return;
                }
            } else if (input.seq > fte.seq) {
                // insert packet
                // std::cout << "seq: " << (unsigned)input.seq << ", expected: " << (unsigned)fte.seq << "\n";
                unsigned _BitInt(9) new_node_ptr;
                dymem_new(&input, &new_node_ptr);
                int slow_cnt = fte.slow_cnt;
                dymem_t head;
                dymem_t tail;
                dymem_lookup((unsigned _BitInt(9)*)&(fte.pointer), &head); // lookup tail
                dymem_lookup(&fte.pointer2, &tail); // lookup tail
                unsigned _BitInt(9) node_ptr = fte.pointer;
                if (slow_cnt != 0) {
                    if (input.seq >= tail.meta.seq + tail.meta.len) {
                        dymem_update(&fte.pointer2, &new_node_ptr);
                        fte.pointer2 = new_node_ptr;
                    } else if (input.seq + input.len <= head.meta.seq) {
                        dymem_update(&new_node_ptr, (unsigned _BitInt(9)*)&(fte.pointer)); //new_node_ptr -> next = fte.pointer
                        fte.pointer = new_node_ptr;
                    } else {
                        // std::cout << "input seq: " << (unsigned)input.seq << ", len: " << (unsigned)input.len << "\n";
                        // std::cout << "head seq: " << (unsigned)head.meta.seq << ", len: " << (unsigned)head.meta.len << "\n";
                        // std::cout << "tail seq: " << (unsigned)tail.meta.seq << ", len: " << (unsigned)tail.meta.len << "\n";
                        // int i = 0;
                        while (true) {
                            if (input.seq < head.meta.seq + head.meta.len) {
                                //overlap packet, drop
                                input.pkt_flags = PKT_DROP;
                                primate_unlock(&(input.tuple));
                                top_intf.Output_simple<meta_t>(input);
                                return;
                            } else {
                                dymem_t next_node;
                                dymem_lookup(&head.next, &next_node);
                                // if (i < 200) {
                                //     std::cout << "next seq: " << (unsigned)next_node.meta.seq << ", len: " << (unsigned)next_node.meta.len << ", slow_cnt: " << slow_cnt << "\n";
                                //     i++;
                                // }
                                if ((--slow_cnt) == 0) {
                                    // insert to tail
                                    dymem_update(&node_ptr, &new_node_ptr);
                                    fte.pointer2 = new_node_ptr;
                                    break;
                                } else if (input.seq + input.len > next_node.meta.seq) {
                                    node_ptr = head.next;
                                    head = next_node;
                                } else {
                                    // insert
                                    dymem_update(&node_ptr, &new_node_ptr);
                                    dymem_update(&new_node_ptr, &head.next);
                                    break;
                                }
                            }
                        }
                    }
                }
                fte.slow_cnt ++;
                flow_table_update(&fte);
                primate_unlock(&(input.tuple));
                return;
            } else {
                input.pkt_flags = PKT_DROP;
                primate_unlock(&(input.tuple));
                top_intf.Output_simple<meta_t>(input);
                return;
            }
        } else {
            // Flow doesn't exist, insert
            flow_table_insert(&input);
            primate_unlock(&(input.tuple));
            top_intf.Output_simple<meta_t>(input);
            return;
        }
    }
}

