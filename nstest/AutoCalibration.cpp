#include "net_impls.h"
#include "vartypes.h"
#include "os_util.hpp"
#include "log.h"
#include "icom/compiler.h"

#include "icom/posix_thread.h"
#include "icom/posix_wait.h"

#include <stdint.h>
#include <thread>
#include <mutex>

#include "drive_unit.h"
#include "wheel.h"

void hexdump(const unsigned char *memory, int cb) {
    int i = 0;
    while (i < cb) {
        printf("%02X ", memory[i]);
        if ( i > 0 && i % 8 == 0) {
            printf("\t");
        }else {
            if (i > 0 && i % 16 == 0) {
                printf("\n");
            }else{
                printf(" ");
            }
        }
        ++i;
    }
}

int init(const char *target_epstr) {
    int robot = mn::init_net();
    if (robot < 0) {
        return -1;
    }

    // login mt with Calibration identity
    int retval = mn::login_to_host(robot, target_epstr, kControlorType_Calibration);
    if (retval < 0) {
        mn::disconnect_host(robot);
        return -1;
    }

    return robot;
}

std::mutex lock_wheels;
std::vector<int> all_wheels;

std::vector<struct mn::var_item> all_vars;

void on_wheels_of_driveunit(uint32_t igr, const void *p) {
    mn::asio_t *asio = (struct mn::asio_t *)p;
    if (asio->err_ < 0) {
        printf("get wheels of unit failed.\n");
        return;
    }
    printf("get wheels of unit.\n");

    mn::wheels_of_driveunit *wheels = (mn::wheels_of_driveunit *)p;
    printf("wheels id of drive unit : %d\n", wheels->unit_id);
    for (auto &iter : wheels->wheels) {
        printf("%d\t", iter);
        lock_wheels.lock();
        all_wheels.push_back(iter);
        lock_wheels.unlock();
    }
    printf("\n");
}

int load_driveunit(int robot, std::vector<int> &ids_driveunit) { 
    int retval;
    nsp::os::waitable_handle w;

    // query all working var, screening data of type @kVarType_DriveUnit
    retval = mn::post_dbg_varls_request(robot, [&](uint32_t, const void *p) {
        mn::asio_t *asio = (mn::asio_t *)p;
        retval = asio->err_;
        if (retval >= 0) {
            mn::var_list *varls = (mn::var_list *)p;

            // save all var item
            all_vars = varls->items_;
            for (auto &iter : varls->items_) {
                if (iter.type_ == kVarType_DriveUnit) {
                    ids_driveunit.push_back(iter.id_);
                }
            }
        }
        w.sig();
    });
    w.wait();

    if (0 == ids_driveunit.size()) {
        lowarn("ac") << "no dirve unit current existed?";
        return -1;
    }

    struct mn::common_title title;
    for (auto &iter : ids_driveunit) {
        struct mn::common_title_item item;
        item.varid = iter;
        item.vartype = kVarType_DriveUnit;
        item.offset = 0;
        item.length = sizeof(var__drive_unit_t);
        title.items.push_back(item);
    }
     
    // get drive unit information by common read interface
    retval = mn::post_common_read_request_by_id(robot, title, [&](uint32_t, const void *p) {
        mn::asio_t *asio = (mn::asio_t *)p;
        retval = asio->err_;
        if (retval >= 0) {
            struct mn::common_data *data = (struct mn::common_data *)p;
            for (auto &iter : data->items) {
                printf("var:%d\n", iter.varid);
                var__drive_unit_t *du = (var__drive_unit_t *)iter.data.data();
                printf("install:%.2f, %.2f, %.2f\n", du->install_.x_, du->install_.y_, du->install_.w_);
                printf("calibrated_:%.2f, %.2f, %.2f\n", du->calibrated_.x_, du->calibrated_.y_, du->calibrated_.w_);

                // query all wheels which binding in this unit
                mn::query_wheels_by_driveunit(robot, iter.varid, &on_wheels_of_driveunit);
            }
        }
        w.sig();
    });
    w.wait();
    return retval;
}

void test_calibrated_dwheel(int robot) {
    for (auto &iter : all_vars) {
        if (iter.type_ == kVarType_DWheel) {
            mn::common_data data;
            
            struct mn::common_data_item item;
            item.varid = iter.id_;
            item.offset = offsetof(var__dwheel_t, slide_weight_);

            double slide_weight = 2109.1901;
            item.data += (char *)&slide_weight;

            data.items.push_back(item);

            nsp::os::waitable_handle w;
            mn::post_common_calibate_request_by_id(robot, data, [&](uint32_t, const void *p) {
                mn::asio_t *asio = (mn::asio_t *)p;
                if (asio->err_ >= 0 ){
                    printf("calibrate dwheel %d completed.\n", iter.id_);
                }else{
                    printf("calibrate error! %d\n", asio->err_);
                }
                
                w.sig();
            });
            w.wait();
        }
    }
}

void test_loc_memory(int robot) {
    mn::post_localization_cfgread_request(robot, [&](uint32_t, const void *p) {
        mn::asio_t *asio = (mn::asio_t *)p;
        if (asio->err_ >= 0 ){
            struct mn::loc_data_t *loc = (struct mn::loc_data_t *)p;
            hexdump(loc->data_, sizeof(loc->data_));

            // write data to loc cofig memory area
            mn::post_localization_cfgwrite_request(robot, (const unsigned char *)"abcd#1234", 0, 7, [](uint32_t, const void *p) {});
        }else{
            printf("calibrate error! %d\n", asio->err_);
        }
    });
}

int main(int argc,char **argv) {
    char target_epstr[16];
    if (argc <= 1) {
        strcpy(target_epstr, "127.0.0.1:4409");
    }else{
        strcpy(target_epstr, argv[1]);
    }

    // initialize and login mt host
    int robot = init(target_epstr);
    if (robot < 0) {
        return 1;
    }

    // get data of all unit, return the id list of unit by @ids_driveunit
    std::vector<int> ids_driveunit;
    load_driveunit(robot, ids_driveunit);

    // sim wait, until wheels query completed.
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // test the calibrate operation
    test_calibrated_dwheel(robot);

    nsp::os::pshang();
    return 0;
}