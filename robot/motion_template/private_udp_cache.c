/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "version.h"

#if !_WIN32

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

#include "private_udp_cache.h"

#pragma pack(push, 1)

struct cache_node_t {
    int access; 
    int using; 
    uint64_t  weight;       
    unsigned char buffer[CACHE_PRE_PIECES_SIZE];
};

struct cache_pool {
    struct cache_node_t nodes[CACHE_PIECES_COUNT];
};

#pragma pack(pop)

static struct cache_pool *cache = NULL;
static pthread_mutex_t pc_locker = PTHREAD_MUTEX_INITIALIZER;

int select_cache_memory(int access, unsigned char **buffer) {
    int i;
    int retval;

    if (!buffer) {
        return -1;
    }
    
    pthread_mutex_lock(&pc_locker);

    if (!cache){
        cache = (struct cache_pool *)malloc(sizeof(struct cache_pool));
        bzero(cache, sizeof(struct cache_pool));
    }

    retval= -1;
    i = 0;
    while (i < CACHE_PIECES_COUNT) {
        if (access == cache->nodes[i].access) {
            *buffer = &cache->nodes[i].buffer[0];
            retval = i;
            break;
        }
        i++;
    }

    pthread_mutex_unlock(&pc_locker);
    return retval;
}

int select_cache_memory_dec(int access, unsigned char **buffer) {
    int i;
    int min_weight_index;
    uint64_t current_minimum_weitht = (uint64_t)(~0);

    if (!buffer || !cache) {
        return -1;
    }
    
    min_weight_index = -1;
    pthread_mutex_lock(&pc_locker);
    for (i = 0; i < CACHE_PIECES_COUNT; i++) {
        if (access == cache->nodes[i].access) {
            if (cache->nodes[i].weight < current_minimum_weitht) {
                current_minimum_weitht = cache->nodes[i].weight;
                min_weight_index = i;
            }
        }
    }
    if (min_weight_index >= 0) {
        *buffer = &cache->nodes[min_weight_index].buffer[0];
    }
    pthread_mutex_unlock(&pc_locker);
    
    return min_weight_index;
}

void update_cache_memory(int pic_id, int update_size,uint64_t update_weight, int access_set) {
    pthread_mutex_lock(&pc_locker);
    if (pic_id < CACHE_PIECES_COUNT) {
        cache->nodes[pic_id].using = update_size;
        cache->nodes[pic_id].weight = update_weight;
        cache->nodes[pic_id].access = access_set;
        // posix__atomic_xchange(&cache->nodes[pic_id].using, update_size);
        // posix__atomic_xchange(&cache->nodes[pic_id].weight, update_weight);
        // posix__atomic_xchange(&cache->nodes[pic_id].access, access_set);
    }
    pthread_mutex_unlock(&pc_locker);
}

#endif // !_WIN32