#include "pstorage.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "logger.h"
#include "posix_ifos.h"
#include "posix_string.h"
#include "posix_thread.h"
#include "posix_time.h"

#include "vartypes.h"

#pragma pack(push,1)

struct period_storage_data {
	void *mptr;
	uint32_t mlen;
};

#pragma pack(pop)

#define RECORD_DATA_FILE "record.dat"

static struct period_storage_data mapped_data = {.mptr = NULL, .mlen = 0 };

int run__load_mapping() {
    char path[260];
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
	
    return retval;
}

void run__release_mapping() {
	if (mapped_data.mptr && mapped_data.mlen > 0 ) {
		munmap(mapped_data.mptr, sizeof (mapped_data.mlen));
	}
}

int run__write_mapping(uint32_t len, const void *data) {
	if (!mapped_data.mptr || (mapped_data.mlen <= 0) || !data) {
		return -EINVAL;
	}
	
	memcpy(mapped_data.mptr, data, len);
	return 0;
}

int run__read_mapping(uint32_t offset, uint32_t len, void *data) {
	if (!mapped_data.mptr || (mapped_data.mlen <= 0) || (offset + len > mapped_data.mlen) || !data) {
		return -EINVAL;
	}
	
	memcpy(data, mapped_data.mptr + offset, len);
	return 0;
}