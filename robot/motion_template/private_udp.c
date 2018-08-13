/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "version.h"

#if !_WIN32

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "private_udp_cache.h"
#include "posix_atomic.h"
#include "posix_time.h"
#include "private_udp.h"
#include "logger.h"

#define MTU_OVERFLOW (1600)

struct ncb_t {
    int sockfd;

    pthread_attr_t thread_attr;
    pthread_t thread_rcv;

    pthread_attr_t thread_handler_attr;
    pthread_t thread_handler;
    pthread_condattr_t condattr_handler;
    pthread_cond_t cond_handler;
    pthread_mutexattr_t mtxattr_handler;
    pthread_mutex_t mtx_handler;
    int pass_handler;

    struct sockaddr_in local_addr;

    private_udp_callback_t callback;
};

static void ncbclose(struct ncb_t *ncb) {
    if (ncb) {
        if (ncb->sockfd > 0) {
            shutdown(ncb->sockfd, SHUT_RDWR);
            close(ncb->sockfd);
            ncb->sockfd = -1;
        }

        /*
        pthread_attr_destroy(&ncb->thread_attr);
        pthread_attr_destroy(&ncb->thread_control_attr);
        pthread_attr_destroy(&ncb->thread_handler_attr);
        
        pthread_mutexattr_destroy(&ncb->mtxattr_handler);
         */
    }
}

#define MODULE_DATA_PORT (0x5000)

static struct ncb_t local_ncb;
static struct sockaddr_in module_addr;

static void *routine_handler(void *p) {
    struct ncb_t *ncb = (struct ncb_t *) p;
    int retval;
    int pic_id;
    unsigned char *buffer;
    private_udp_callback_t callback = (private_udp_callback_t) ncb->callback;

    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "private UDP handler thread startup.\n");

    while (1) {
        pthread_mutex_lock(&ncb->mtx_handler);
        while (!ncb->pass_handler) {
            retval = pthread_cond_wait(&ncb->cond_handler, &ncb->mtx_handler);
            if (0 != retval) {
                break;
            }
        }
        ncb->pass_handler = 0;
        pthread_mutex_unlock(&ncb->mtx_handler);

        while ((pic_id = select_cache_memory_dec(kCameraCacheAccess_HandlerReadable, &buffer)) >= 0) {
            callback(&buffer[sizeof (int)], *((int *) buffer));
            update_cache_memory(pic_id, 0, 0, kCameraCacheAccess_ReceiverWriteable);
        }
    }

    return NULL;
}

uint64_t getts() {
    uint64_t ptime;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ptime = (uint64_t) ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    return ptime;
}

static int udp_update_opts(int fd) {
    static const int RECV_BUFFER_SIZE = 0xFFFF;
    static const int SEND_BUFFER_SIZE = 0xFFFF;
    struct linger lgr;

    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (const void *) &RECV_BUFFER_SIZE, sizeof (RECV_BUFFER_SIZE));
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, (const void *) &SEND_BUFFER_SIZE, sizeof (SEND_BUFFER_SIZE));
    lgr.l_onoff = 0;
    lgr.l_linger = 1;
    setsockopt(fd, SOL_SOCKET, SO_LINGER, (char *) &lgr, sizeof ( struct linger));
    return 0;
}

static int receiver(unsigned char *cache_ptr) {
    int rcvcb;
    struct sockaddr_in from_addr;
    socklen_t from_len;
    int err;
    char rcvbuf[2048];

    if (!cache_ptr) {
        return -1;
    }

    while (1) {
        from_len = sizeof (from_addr);
        rcvcb = recvfrom(local_ncb.sockfd, rcvbuf, sizeof (rcvbuf), 0, (__SOCKADDR_ARG) & from_addr, (socklen_t * __restrict) & from_len);
        err = errno;
        if (rcvcb < 0) {
            if ((err != EINTR) && (err != EAGAIN)) {
                return -1;
            }
            continue;
        }

        if (rcvcb < sizeof(int) || rcvcb > MTU_OVERFLOW) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                    " private pool recv size large than MTU,rcvcb=%d", rcvcb);
            return -1;
        }

        // cache 布局:
        // [长度][数据]
        memcpy(cache_ptr, &rcvcb, sizeof(rcvcb));
        memcpy(&cache_ptr[sizeof (rcvcb)], rcvbuf, rcvcb);
        break;
    }

    return 0;
}

static void *routine_rcv(void *p) {
    unsigned char *cache_ptr;
    int pic_id = -1;
    int retval = 0;

    while (1) {
        if (retval >= 0) {
            pic_id = select_cache_memory(kCameraCacheAccess_ReceiverWriteable, &cache_ptr);
            if (pic_id < 0) {
                log__save("motion_template", kLogLevel_Warning, kLogTarget_Filesystem | kLogTarget_Stdout, 
					" private UDP pool, non free cache can be use for receiver.\n");
                usleep(10 * 1000);
                continue;
            }
        }

        retval = receiver(cache_ptr);
        if (retval >= 0) {
            update_cache_memory(pic_id, *((int *) cache_ptr), posix__clock_gettime(), kCameraCacheAccess_HandlerReadable);

            pthread_mutex_lock(&local_ncb.mtx_handler);
            local_ncb.pass_handler = 1;
            pthread_cond_signal(&local_ncb.cond_handler);
            pthread_mutex_unlock(&local_ncb.mtx_handler);
        }
    }
    return NULL;
}

static int create_sockfd(const char *local_addr, uint16_t port, struct sockaddr_in *output) {
    int fd;
    int optval;
    int retval;
    struct sockaddr_in addr;
    socklen_t addrlen;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
			"private UPD:failed create services socket.errno=%d.\n", errno);
        return -1;
    }

    optval = 1;
    retval = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof ( optval));

    addr.sin_addr.s_addr = INADDR_ANY;
    if (local_addr) {
        addr.sin_addr.s_addr = inet_addr(local_addr);
    }
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    retval = bind(fd, (struct sockaddr *) &addr, sizeof ( struct sockaddr));
    if (retval < 0) {
        close(fd);
        return -1;
    }

    /* setsockopt */
    if (udp_update_opts(fd) < 0) {
        close(fd);
        return -1;
    }

    getsockname(fd, (struct sockaddr *) output, &addrlen);
    return fd;
}

int private_udp_startup(const char *module_address, private_udp_callback_t callback) {

    if (!callback || !module_address) {
        return -EINVAL;
    }

    static int inited = 0;
    if (1 != posix__atomic_inc(&inited)) {
        return -1;
    }
    bzero(&local_ncb, sizeof (local_ncb));

    local_ncb.sockfd = create_sockfd(NULL, MODULE_DATA_PORT, &local_ncb.local_addr);
    if (local_ncb.sockfd < 0) {
        ncbclose(&local_ncb);
        return -1;
    }

    local_ncb.callback = callback;

    module_addr.sin_addr.s_addr = inet_addr(module_address);
    module_addr.sin_family = AF_INET;
    module_addr.sin_port = htons(MODULE_DATA_PORT);

    pthread_attr_init(&local_ncb.thread_attr);
    pthread_create(&local_ncb.thread_rcv, &local_ncb.thread_attr, &routine_rcv, NULL);

    pthread_mutexattr_init(&local_ncb.mtxattr_handler);
    pthread_mutexattr_settype(&local_ncb.mtxattr_handler, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&local_ncb.mtx_handler, &local_ncb.mtxattr_handler);
    pthread_condattr_init(&local_ncb.condattr_handler);
    pthread_condattr_setclock(&local_ncb.condattr_handler, CLOCK_MONOTONIC);
    pthread_cond_init(&local_ncb.cond_handler, &local_ncb.condattr_handler);
    local_ncb.pass_handler = 0;
    pthread_attr_init(&local_ncb.thread_handler_attr);
    pthread_create(&local_ncb.thread_rcv, &local_ncb.thread_handler_attr, &routine_handler, (void *__restrict) & local_ncb);
    return 0;
}

int private_udp_write(const void *buffer, int cb) {
    return sendto(local_ncb.sockfd, (const void *) buffer, cb, 0, (__CONST_SOCKADDR_ARG) & module_addr, sizeof (module_addr));
}

#endif // ARM_LINUX
