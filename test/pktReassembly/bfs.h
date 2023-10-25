#include "primate.h"

int flow_table_read(meta_t *input, fce_t *fte);
int flow_table_delete(fce_t *fte);
int flow_table_update(fce_t *fte);
int flow_table_insert(meta_t *input);
int dymem_lookup(unsigned _BitInt(9) *ptr, dymem_t *pkt);
int dymem_new(meta_t *input, unsigned _BitInt(9) *ptr);
int dymem_free(unsigned _BitInt(9) *ptr);
int dymem_update(unsigned _BitInt(9) *ptr, unsigned _BitInt(9) *next_ptr);
int primate_lock(flow_key_t *key);
int primate_unlock(flow_key_t *key);