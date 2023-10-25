#include "bfs.h"
#include <map>
#include <vector>

using namespace std;

static map<flow_key_t, fce_t> flow_table;
static vector<dymem_t> mem(512);
static bool mem_valid[512];

int flow_table_read(meta_t *input, fce_t *fte) {
    flow_key_t key = input->tuple;
    if (flow_table.find(key) != flow_table.end()) {
        *fte = flow_table[key];
    } else {
        fte->ch0_bit_map = 0;
    }
    return 0;
}

int flow_table_delete(fce_t *fte) {
    flow_key_t key = fte->tuple;
    flow_table.erase(key);
    return 0;
}

int flow_table_update(fce_t *fte) {
    flow_key_t key = fte->tuple;
    flow_table[key] = *fte;
    return 0;
}

int flow_table_insert(meta_t *input) {
    if ((input->tcp_flags & (1 << TCP_FIN) | (input->tcp_flags & (1 << TCP_RST))) == 0) {
        flow_key_t key = input->tuple;
        fce_t tmp;
        tmp.tuple = input->tuple;
        if ((input->tcp_flags & (1 << TCP_SYN)) != 0) {
            tmp.seq = input->seq + 1;
        } else {
            tmp.seq = input->seq + input->len;
        }
        tmp.pointer = 0;
        tmp.slow_cnt = 0;
        tmp.pointer2 = 0;
        tmp.ch0_bit_map = 1;
        flow_table[key] = tmp;
    }
    return 0;
}

int dymem_lookup(unsigned _BitInt(9) *ptr, dymem_t *pkt) {
    int idx = *ptr;
    *pkt = mem[idx];
    return 0;
}

int dymem_new(meta_t *input, unsigned _BitInt(9) *ptr) {
    dymem_t tmp;
    tmp.meta = *input;
    tmp.next = 0;
    for (int i = 0; i < 512; i++) {
        if (!mem_valid[i]) {
            *ptr = i;
            mem[i] = tmp;
            mem_valid[i] = true;
            return 0;
        }
    }
    cout << "dymem overflow\n";
    return 0;
}

int dymem_free(unsigned _BitInt(9) *ptr) {
    int idx = *ptr;
    mem_valid[idx] = false;
    return 0;
}

int dymem_update(unsigned _BitInt(9) *ptr, unsigned _BitInt(9) *next_ptr) {
    int idx = *ptr;
    mem[idx].next = *next_ptr;
    return 0;
}

int primate_lock(flow_key_t *key) {
    return 0;
}

int primate_unlock(flow_key_t *key) {
    return 0;
}
