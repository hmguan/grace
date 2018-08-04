#include "pstorage.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include "logger.h"
#include "posix_ifos.h"
#include "posix_time.h"
#include "posix_string.h"

#include "vartypes.h"
#include "navigation.h"
#include "vehicle.h"
#include "drive_unit.h"
#include "wheel.h"

#if !defined P_STORAGE_FILE_SIZE
#define P_STORAGE_FILE_SIZE     (8192)
#endif

#pragma pack(push,1)

struct period_storage_data {
	void *mptr;
	uint32_t mlen;
};

struct p_storage_formt {
    upl_t upl;
    double last_total_odo;
};

struct p_storage_calibration_node {
    int len;  /* total length of this node, include user data size and size of node struct */
    int id;
    unsigned char data[0];
};

struct p_storage_calibration_describe {
    /* by initialize, this value is zero */
    int count;

    /* tail means length of head to last node's last data byte.
        so, new node insert into queue should use tail as it's begin offset */
    int tail;

    /* head pointer of the var list */
    struct p_storage_calibration_node head[0];
};

struct p_storage_t {
    union {
        struct {
            unsigned char formt[1024];
            unsigned char forloc[1024];
            unsigned char forcab[4096];
        }p_storage_feild;

        unsigned char p_storage_occupy[P_STORAGE_FILE_SIZE];
    };
};

#pragma pack(pop)

#define RECORD_DATA_FILE "record.dat"

static pthread_mutex_t lock_calibration = PTHREAD_MUTEX_INITIALIZER;
static struct period_storage_data mapped_data = {.mptr = NULL, .mlen = 0 };
static int p_storage_retval = -1;

int mm__load_mapping() {
    char path[256];
    int retval;
    int fd;
	struct stat st;
	void *addr;
	
	if (mapped_data.mptr || mapped_data.mlen > 0 ) {
		return -1;
	}

    retval = -1;
    fd = -1;
    
    do {
#if _WIN32
		posix__sprintf(path, cchof(path), "%s\\%s", posix__getpedir(), RECORD_DATA_FILE);
#else
		posix__sprintf(path, cchof(path), "%s/%s", posix__getpedir(), RECORD_DATA_FILE);
#endif
		fd = open(path, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
        if (fd < 0) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "failed to open associated file %s for total odo meter mapping.error=%d", path, errno);
            break;
        }
		
		if (stat(path, &st) < 0) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "failed to get file [%s] stat. error=%d", path, errno);
			break;
		}
		
		if(st.st_size < P_STORAGE_FILE_SIZE) {
            char empty[P_STORAGE_FILE_SIZE];
            memset(&empty, 0, sizeof(empty));
			retval = write(fd, &empty, sizeof(empty));
			if(retval <= 0) {
                log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "failed to init mapping file for UPL. error=%d", errno);
                break;
			}
		} 

        /* pa_offset = offset & ~(sysconf(_SC_PAGE_SIZE) - 1) */
        addr = mmap(NULL, P_STORAGE_FILE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (MAP_FAILED == addr) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "failed to map mapping file into VIRT. error=%d", errno);
            break;
        }

        close(fd);
        fd = -1;
        mapped_data.mptr = addr;
		mapped_data.mlen = P_STORAGE_FILE_SIZE;

        /* all ok */
        retval = 0;
		log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,"successful mapped file for UPL, [%s]", path);
    } while (0);
	
	if (fd >= 0) {
        close(fd);
    }
	
    p_storage_retval = retval;
    return retval;
}

void mm__release_mapping() {
	if (mapped_data.mptr && mapped_data.mlen > 0 && p_storage_retval >= 0) {
		munmap(mapped_data.mptr, sizeof (mapped_data.mlen));
	}
}

int mm__getupl(void *upl) {
    struct p_storage_formt *formt;
    struct p_storage_t *pmap;

    if (!mapped_data.mptr || 0 == mapped_data.mlen || p_storage_retval < 0) {
        return -EINVAL;
    }

    pmap = (struct p_storage_t *)mapped_data.mptr;
    formt = (struct p_storage_formt *)&pmap->p_storage_feild.formt[0];

    memcpy(upl, &formt->upl, sizeof(formt->upl));
    return 0;
}

int mm__setupl(const void *upl) {
    struct p_storage_formt *formt;
    struct p_storage_t *pmap;

    if (!mapped_data.mptr || 0 == mapped_data.mlen || p_storage_retval < 0) {
        return -EINVAL;
    }

    pmap = (struct p_storage_t *)mapped_data.mptr;
    formt = (struct p_storage_formt *)&pmap->p_storage_feild.formt[0];

    memcpy(&formt->upl, upl, sizeof(formt->upl));
    return 0;
}

int mm__getloc(void *loc, int cb) {
    unsigned char *forloc;
    struct p_storage_t *pmap;

    if (!mapped_data.mptr || 0 == mapped_data.mlen || p_storage_retval < 0) {
        return -EINVAL;
    }

    pmap = (struct p_storage_t *)mapped_data.mptr;
    forloc = &pmap->p_storage_feild.forloc[0];

    memcpy(loc, forloc, cb);
    return 0;
}

int mm__setloc(const void *loc, int cb) {
    unsigned char *forloc;
    struct p_storage_t *pmap;

    if (!mapped_data.mptr || 0 == mapped_data.mlen || p_storage_retval < 0) {
        return -EINVAL;
    }

    pmap = (struct p_storage_t *)mapped_data.mptr;
    forloc = &pmap->p_storage_feild.forloc[0];

    memcpy(forloc, loc, cb);
    return 0;
}

static
int mm__insert_calibration_node(struct p_storage_calibration_describe *p_calibration_desc, int varid, int cb, const void *data) {
    struct p_storage_calibration_node *node, *tail;
    unsigned char *p;
    int nodecb;

    if (!p_calibration_desc || !data || cb <= 0) {
        return -EINVAL;
    }

    nodecb = cb + sizeof(struct p_storage_calibration_node);
    if (NULL == (node = (struct p_storage_calibration_node *)malloc(nodecb))) {
        return -ENOMEM;
    }

    pthread_mutex_lock(&lock_calibration);

    p = (unsigned char *)p_calibration_desc->head;

    /* current tail is the previous node of @node */
    p += p_calibration_desc->tail;
    tail = (struct p_storage_calibration_node *)p;
    
    /* build user data */
    node->len = nodecb;
    node->id = varid;
    memcpy(node->data, data, cb);

    /* copy node to the mapped pages */
    memcpy(tail, node, nodecb );

    /* move tail pointer */
    p_calibration_desc->tail += node->len;

    /* increase the total count of nodes */
    ++p_calibration_desc->count;
        
    pthread_mutex_unlock(&lock_calibration);
    free(node);

    return 0;
}

static
int mm__update_calibration_bysearch(struct p_storage_calibration_describe *p_calibration_desc, int varid,int cb, const void *data) {
    struct p_storage_calibration_node *cursor;
    int retval;
    int i;

    if (!p_calibration_desc || !data || cb <= 0) {
        return -EINVAL;
    }

    cursor = p_calibration_desc->head;
    retval = -1;

    pthread_mutex_lock(&lock_calibration);

    /* search the object which specified by varid and type */
    for (i = 0; i < p_calibration_desc->count; i++) {
        if (
            cursor->id == varid &&
             ((cursor->len - sizeof(struct p_storage_calibration_node)) == cb))
        {
            log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,"pstorage recored calibration var %d.", varid);
            memcpy(cursor->data, data, cb);
            retval = 0;
            break;
        }

        cursor = (struct p_storage_calibration_node *)((unsigned char *)cursor + cursor->len);
    }
    pthread_mutex_unlock(&lock_calibration);

    return retval;
}

static
int mm__load_calibration_bysearch(struct p_storage_calibration_describe *p_calibration_desc, int varid, void *data) {
    struct p_storage_calibration_node *cursor;
    int retval;
    int i;

    if (!p_calibration_desc || !data ) {
        return -EINVAL;
    }

    cursor = p_calibration_desc->head;
    retval = -1;

    pthread_mutex_lock(&lock_calibration);

    /* search the object which specified by varid and type */
    for (i = 0; i < p_calibration_desc->count; i++) {
        if ( cursor->id == varid ) {
            log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,"pstorage load calibration var %d.", varid);
            memcpy(data, cursor->data, cursor->len - sizeof(struct p_storage_calibration_node));
            retval = 0;
            break;
        }

        cursor = (struct p_storage_calibration_node *)((unsigned char *)cursor + cursor->len);
    }
    pthread_mutex_unlock(&lock_calibration);

    return retval;
}

int mm__set_calibration(int varid, int cb, const void *data) {
    struct p_storage_calibration_describe *p_calibration_desc;
    struct p_storage_t *pmap;

    if (!mapped_data.mptr || 0 == mapped_data.mlen || !data || cb <= 0 || p_storage_retval < 0 ) {
        return -EINVAL;
    }

    pmap = (struct p_storage_t *)mapped_data.mptr;
    p_calibration_desc = (struct p_storage_calibration_describe *)&pmap->p_storage_feild.forcab[0];

    if (mm__update_calibration_bysearch(p_calibration_desc, varid, cb ,data) < 0) {
        /* object no found, add it, insert to mapping file */
        return mm__insert_calibration_node(p_calibration_desc, varid, cb, data);
    }

    return 0;
}

int mm__get_calibration(int varid, void *data) {
        struct p_storage_calibration_describe *p_calibration_desc;
    struct p_storage_t *pmap;

    if (!mapped_data.mptr || 0 == mapped_data.mlen || !data || p_storage_retval < 0) {
        return -EINVAL;
    }

    pmap = (struct p_storage_t *)mapped_data.mptr;
    p_calibration_desc = (struct p_storage_calibration_describe *)&pmap->p_storage_feild.forcab[0];

    return mm__load_calibration_bysearch(p_calibration_desc, varid, data);
}