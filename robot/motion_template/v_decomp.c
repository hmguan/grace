#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "v_decomp.h"
#include "math.h"
#include "var.h"
#include "logger.h"
#include "PosandTrack.h"
#include "posix_time.h"

#define DATA_W 0

static int chassis_state = 1;
static int sdd_firsttime = 0;
static uint64_t sdd_time_stamp_last = 0;
static double *w_sdd_last;

void EnableDrive_unit(const struct list_head *drive_unit, int aa) {
    struct list_head *pos;
    var__drive_unit_t *dri;
    var__functional_object_t *drive_obj;
    var__functional_object_t **wheel_obj;
    int i=0;

	list_for_each(pos, drive_unit) {
        drive_obj = containing_record(pos, var__functional_object_t, local_);
        dri = var__object_body_ptr(var__drive_unit_t, drive_obj);
        wheel_obj = malloc(dri->associated_cnt_ * sizeof ( void *));
        var__driveunit_parse_to_functional(dri, (void **) wheel_obj, NULL);
        for (i = 0; i < dri->associated_cnt_; i++) {
            if (dri->unit_type_ == kOnlySWheel) {
                if (wheel_obj[i]->type_ == kVarType_SWheel) {
                    var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, wheel_obj[i]);
                    swheel->i.enable_ = aa;
                    var__commit_wheel_dup(wheel_obj[i]);
                }
            } else if (dri->unit_type_ == kOnlyDWheel) {
                if (wheel_obj[i]->type_ == kVarType_DWheel) {
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, wheel_obj[i]);
                    dwheel->i.enable_ = aa;
                    var__commit_wheel_dup(wheel_obj[i]);
                }
            } else if (dri->unit_type_ == kSDWheel) {
                if (wheel_obj[i]->type_ == kVarType_SWheel) {
                    var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, wheel_obj[i]);
                    swheel->i.enable_ = aa;
                    var__commit_wheel_dup(wheel_obj[i]);
                }
                if (wheel_obj[i]->type_ == kVarType_DWheel) {
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, wheel_obj[i]);
                    dwheel->i.enable_ = aa;
                    var__commit_wheel_dup(wheel_obj[i]);
                }
            } else {
                if (wheel_obj[i]->type_ == kVarType_DWheel) {
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, wheel_obj[i]);
                    dwheel->i.enable_ = aa;
                    var__commit_wheel_dup(wheel_obj[i]);
                }
            }
            var__release_wheel_dup(wheel_obj[i]);
        }
        free(wheel_obj);
    }
}

int get_num_of_wheel(const struct list_head *drive_unit, int *num_time_stamp) {
    struct list_head *pos;
    int num_wheel = 0;
    (*num_time_stamp) = 0;
    var__drive_unit_t *dri;

	list_for_each(pos, drive_unit ) {
        var__functional_object_t *drive_obj;
        drive_obj = containing_record(pos, var__functional_object_t, local_);
        dri = var__object_body_ptr(var__drive_unit_t, drive_obj);
        num_wheel++;
        (*num_time_stamp)++;
        if (dri->unit_type_ == kSDDWheel) {
            num_wheel++;
            (*num_time_stamp) += 2;
        }
        if (dri->unit_type_ == kSDWheel) {
            (*num_time_stamp)++;
        }
    }
    return num_wheel;
}

void v_comm_de(double x, double y, double W, double V_x, double V_y, v_wheel *out) {
    double v_x=0, v_y=0;
    double sita = atan2(y, x);
    v_x = -W * sqrt(x * x + y * y) * sin(sita) + V_x;
    v_y = W * sqrt(x * x + y * y) * cos(sita) + V_y;
    out->_v_x = v_x;
    out->_v_y = v_y;
}

int chassis_state_checked(const struct list_head *drive_unit, var__vehicle_t *veh)
{
    double W=0, V_x=0, V_y=0;
    W = veh->i.command_velocity_.w_;
    V_x = veh->i.command_velocity_.x_;
    V_y = veh->i.command_velocity_.y_;
    var__drive_unit_t *dri;
    struct list_head *pos;
    int i = 0; 
    var__functional_object_t *drive_obj;
    var__functional_object_t **wheel_obj;

	list_for_each(pos, drive_unit) {
        double x = 0, y = 0, angle = 0;
        double min_angle = 0, max_angle = 0;
        drive_obj = containing_record(pos, var__functional_object_t, local_);
        dri = var__object_body_ptr(var__drive_unit_t, drive_obj);
        if (dri->unit_type_ == kSDDWheel||dri->unit_type_ == kSDWheel) {
            x = dri->calibrated_.x_ + dri->install_.x_;
            y = dri->calibrated_.y_ + dri->install_.y_;
            angle = dri->calibrated_.theta_ + dri->install_.theta_;
            wheel_obj = malloc(dri->associated_cnt_ * sizeof (void *));
            var__driveunit_parse_to_functional(dri, (void **) wheel_obj, NULL);
            for (i = 0; i < dri->associated_cnt_; i++) {
                if (wheel_obj[i]->type_ == kVarType_SWheel) {
                    var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, wheel_obj[i]);
                    angle = swheel->actual_angle_enc_ / swheel->scale_feedback_ + angle;
                    min_angle = swheel->min_angle_;
                    max_angle = swheel->max_angle_;
                }
                var__release_wheel_dup(wheel_obj[i]);
            }
            free(wheel_obj);
        }
        v_wheel V_temp;
        v_comm_de(x, y, W, V_x, V_y, &V_temp); 
        double alfa_e = atan2(V_temp._v_y, V_temp._v_x); 
        if (fabs(V_temp._v_x) < 0.001 && fabs(V_temp._v_y) < 0.001) {
            chassis_state = 1;
            return 0;
        }
        if (alfa_e > max_angle)
        {
            alfa_e = NormalAngle(alfa_e - 3.1415926);
        } else {
            if (alfa_e < min_angle) {
                alfa_e = NormalAngle(alfa_e + 3.1415926);
            }
        }
        if (fabs(NormalAngle(alfa_e - angle))>(veh->steer_angle_error_tolerance_))
        {
            chassis_state = 0;
            return 0;
        } else {
            chassis_state = 1;
        }
        i++;
    }
    return 0;
}

void first_v_decomposite(v_wheel *v_w, const struct list_head *drive_unit, var__vehicle_t *veh) {
    uint64_t time_current = (uint64_t) (posix__clock_gettime() / 10000.0);
    if (sdd_firsttime == 0) {
        sdd_time_stamp_last = time_current;
        sdd_firsttime = 1;
    }
    double deta_time = (double) (time_current - sdd_time_stamp_last) / 1000.0;
    sdd_time_stamp_last = time_current;

    double W=0, V_x=0, V_y=0;
    W = veh->i.command_velocity_.w_;
    V_x = veh->i.command_velocity_.x_;
    V_y = veh->i.command_velocity_.y_;
    chassis_state_checked(drive_unit, veh);
    var__drive_unit_t *dri;
    struct list_head *pos;
    int i = 0; 
	int drive_index = 0;
    int num = 0;
    var__functional_object_t *drive_obj;
    var__functional_object_t **wheel_obj;

	list_for_each(pos, drive_unit) {
        double x = 0, y = 0, angle = 0, roll_wt = 0, slide_wt = 0, roll_wt1 = 0, slide_wt1 = 0, gauge = 0;
        double control_cp = 0;
        drive_obj = containing_record(pos, var__functional_object_t, local_);
        dri = var__object_body_ptr(var__drive_unit_t, drive_obj);
        x = dri->calibrated_.x_ + dri->install_.x_;
        y = dri->calibrated_.y_ + dri->install_.y_;
        angle = dri->calibrated_.theta_ + dri->install_.theta_;
        wheel_obj = malloc(dri->associated_cnt_ * sizeof ( void *));
        var__driveunit_parse_to_functional(dri, (void **) wheel_obj, NULL);
        int d1_or_d2 = 0;
        for (i = 0; i < dri->associated_cnt_; i++) {
            if (dri->unit_type_ == kOnlySWheel) {
                if (wheel_obj[i]->type_ == kVarType_SWheel) {
                    v_w[num]._swheel_obj = wheel_obj[i];
                    var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, wheel_obj[i]);
                    angle = swheel->actual_angle_enc_ / swheel->scale_feedback_ + angle;
                    v_w[num]._min_angle = swheel->min_angle_;
                    v_w[num]._max_angle = swheel->max_angle_;
					v_w[num]._zero_angle = dri->calibrated_.theta_;
                    roll_wt = 0;
                    slide_wt = 0;
                }
				else
				{
					var__release_wheel_dup(wheel_obj[i]);
				}
            } else if (dri->unit_type_ == kOnlyDWheel) {
                if (wheel_obj[i]->type_ == kVarType_DWheel) {
                    v_w[num]._dwheel_1_obj = wheel_obj[i];
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, wheel_obj[i]);
                    v_w[num].max_speed = dwheel->max_speed_;
                    roll_wt = dwheel->roll_weight_;
                    slide_wt = dwheel->slide_weight_;
                }
				else
				{
					var__release_wheel_dup(wheel_obj[i]);
				}
            } else if (dri->unit_type_ == kSDWheel) {
                if (wheel_obj[i]->type_ == kVarType_SWheel) {
                    v_w[num]._swheel_obj = wheel_obj[i];
                    var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, wheel_obj[i]);
                    angle = swheel->actual_angle_enc_ / swheel->scale_feedback_ + angle;
                    v_w[num]._min_angle = swheel->min_angle_;
                    v_w[num]._max_angle = swheel->max_angle_;
					v_w[num]._zero_angle = dri->calibrated_.theta_;
                }
                else if (wheel_obj[i]->type_ == kVarType_DWheel) {
                    v_w[num]._dwheel_1_obj = wheel_obj[i];
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, wheel_obj[i]);
                    v_w[num].max_speed = dwheel->max_speed_;
                    roll_wt = dwheel->roll_weight_;
                    slide_wt = dwheel->slide_weight_;
                }
				else
				{
					var__release_wheel_dup(wheel_obj[i]);
				}
			}
			else if (dri->unit_type_ == kSDDWheel){
                if (wheel_obj[i]->type_ == kVarType_SWheel) {
                    v_w[num]._swheel_obj = wheel_obj[i];
                    var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, wheel_obj[i]);
                    control_cp = swheel->control_cp_;
                    angle = swheel->actual_angle_enc_ / swheel->scale_feedback_ + angle;
                    v_w[num]._min_angle = swheel->min_angle_;
                    v_w[num]._max_angle = swheel->max_angle_;
					v_w[num]._zero_angle = dri->calibrated_.theta_;
                }
                else if (wheel_obj[i]->type_ == kVarType_DWheel && d1_or_d2 == 1) {
                    v_w[num]._dwheel_2_obj = wheel_obj[i];
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, wheel_obj[i]);
                    v_w[num].max_speed = dwheel->max_speed_;
                    roll_wt1 = dwheel->roll_weight_;
                    slide_wt1 = dwheel->slide_weight_;
                }
                else if (wheel_obj[i]->type_ == kVarType_DWheel && d1_or_d2 == 0) {
                    v_w[num]._dwheel_1_obj = wheel_obj[i];
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, wheel_obj[i]);
                    v_w[num].max_speed = dwheel->max_speed_;
                    roll_wt = dwheel->roll_weight_;
                    slide_wt = dwheel->slide_weight_;
                    d1_or_d2 = 1;
                }
                else if (wheel_obj[i]->type_ == kVarType_SDDExtra) {
                    v_w[num]._sddwheel_obj = wheel_obj[i];
                    var__sdd_extra_t *sddwheel = var__object_body_ptr(var__sdd_extra_t, wheel_obj[i]);
                    gauge = sddwheel->gauge_;
                    var__release_wheel_dup(wheel_obj[i]);
                }
				else
				{
					var__release_wheel_dup(wheel_obj[i]);
				}
            }
			else
			{
				var__release_wheel_dup(wheel_obj[i]);
			}
        }
        angle = NormalAngle(angle);
        if (!(dri->unit_type_ == kSDDWheel)) {
            v_wheel v_temp;
            v_comm_de(x, y, W, V_x, V_y, &v_temp);
            v_w[num]._w_type = dri->unit_type_;
            v_w[num].index_drive = drive_index;
            v_w[num]._x = x;
            v_w[num]._y = y;
            v_w[num]._v_x = v_temp._v_x;
            v_w[num]._v_y = v_temp._v_y;
            v_w[num]._ae = atan2(v_temp._v_y, v_temp._v_x);
            v_w[num]._af = angle;
        	if (dri->unit_type_ == kSDWheel)
			{
				v_w[num]._vp = v_w[num]._v_x*cos(v_w[num]._ae) + v_w[num]._v_y*sin(v_w[num]._ae);
			}
			else
			{
				v_w[num]._vp = v_w[num]._v_x*cos(angle) + v_w[num]._v_y*sin(angle);
			}
            v_w[num]._roll_wt = roll_wt;
            v_w[num]._slide_wt = slide_wt;
            if (dri->unit_type_ == kSDWheel) {
				if (chassis_state == 0)
				{
					v_w[num]._vp = 0;
					veh->i.command_velocity_.x_ = 0;
					veh->i.command_velocity_.y_ = 0;
					veh->i.command_velocity_.w_ = 0;
				}
				if (v_w[num]._ae > v_w[num]._max_angle)
				{
					if (NormalAngle(v_w[num]._ae - 3.1415926)<v_w[num]._min_angle)
					{
						v_w[num]._ae = v_w[num]._max_angle;
					}
					else
					{
						v_w[num]._ae = NormalAngle(v_w[num]._ae - 3.1415926);
						v_w[num]._vp = -v_w[num]._vp;
					}
				}
				else
				{
					if (v_w[num]._ae < v_w[num]._min_angle)
					{
						if (NormalAngle(v_w[num]._ae + 3.1415926)>v_w[num]._max_angle)
						{
							v_w[num]._ae = v_w[num]._min_angle;
						}
						else
						{
							v_w[num]._ae = NormalAngle(v_w[num]._ae + 3.1415926);
							v_w[num]._vp = -v_w[num]._vp;
						}
					}
				}
				if (fabs(V_x) < 0.001&&fabs(V_y) < 0.001&&fabs(W) < 0.001)
				{
					v_w[num]._ae = 0;
				}
			}
        } else {
            double V_wx = 0, V_wy = 0, W_w = 0;
            v_wheel V_temp;
            v_comm_de(x, y, W, V_x, V_y, &V_temp);		
            double alfa_e = atan2(V_temp._v_y, V_temp._v_x); 
            if (alfa_e > v_w[num]._max_angle)
            {
                alfa_e = NormalAngle(alfa_e - 3.1415926);
            } else {
                if (alfa_e < v_w[num]._min_angle) {
                    alfa_e = NormalAngle(alfa_e + 3.1415926);
                }
            }
            if (chassis_state == 0)
            {
                V_wx = 0;
                V_wy = 0;
                W_w = 0;
                veh->i.command_velocity_.x_ = V_wx;
                veh->i.command_velocity_.y_ = V_wy;
                veh->i.command_velocity_.w_ = W_w;
            } else {
                V_wx = V_x;
                V_wy = V_y;
                W_w = W;
            }
            double x0, x1, y0, y1;
            x0 = -0.5 * gauge * sin(angle) + x; 
            y0 = 0.5 * gauge * cos(angle) + y;
            x1 = 0.5 * gauge * sin(angle) + x; 
            y1 = -0.5 * gauge * cos(angle) + y;
            v_wheel v_temp;
            v_comm_de(x0, y0, W_w, V_wx, V_wy, &v_temp); 
            v_w[num]._w_type = dri->unit_type_;
            v_w[num].index_drive = drive_index;
            v_w[num].left_or_right = 1; 
            v_w[num]._x = x0;
            v_w[num]._y = y0;
            v_w[num]._v_x = v_temp._v_x;
            v_w[num]._v_y = v_temp._v_y;
            v_w[num]._af = angle;
            v_w[num]._ae = alfa_e;
            v_w[num]._vp = v_w[num]._v_x * cos(angle) + v_w[num]._v_y * sin(angle);
            double w_sdd = ((alfa_e - angle) >= 0 ? 1 : -1) * sqrt(2 * control_cp * max(fabs(alfa_e - angle) - 0.05, 0));
            w_sdd = ((alfa_e - angle) >= 0 ? 1 : -1) * max(fabs((alfa_e - angle) * control_cp), fabs(w_sdd));
            if (fabs(V_temp._v_x) < 0.001 && fabs(V_temp._v_y) < 0.001) {
                w_sdd = 0;
            }
            w_sdd = trim_solo_only_acc(w_sdd, w_sdd_last[num], 3 * deta_time);
            w_sdd_last[num] = w_sdd;
            v_w[num]._vp = v_w[num]._vp - w_sdd * gauge / 2.0;
            v_w[num]._roll_wt = roll_wt;
            v_w[num]._slide_wt = slide_wt;
            v_w[num]._gauge = gauge;

            num = num + 1;
            v_comm_de(x1, y1, W_w, V_wx, V_wy, &v_temp);
            v_w[num]._w_type = dri->unit_type_;
            v_w[num].index_drive = drive_index;
            v_w[num].left_or_right = 0; 
            v_w[num]._x = x1;
            v_w[num]._y = y1;
            v_w[num]._v_x = v_temp._v_x;
            v_w[num]._v_y = v_temp._v_y;
            v_w[num]._ae = alfa_e;
            v_w[num]._af = angle;
            v_w[num]._vp = v_w[num]._v_x * cos(angle) + v_w[num]._v_y * sin(angle);
            v_w[num]._vp = v_w[num]._vp + w_sdd * gauge / 2;
            v_w[num]._roll_wt = roll_wt1;
            v_w[num]._slide_wt = slide_wt1;
            v_w[num]._gauge = gauge;
            v_w[num].max_speed = v_w[num - 1].max_speed;
            v_w[num]._dwheel_1_obj = v_w[num - 1]._dwheel_1_obj;
            v_w[num]._dwheel_2_obj = v_w[num - 1]._dwheel_2_obj;
            v_w[num]._min_angle = v_w[num - 1]._min_angle;
            v_w[num]._max_angle = v_w[num - 1]._max_angle;
        }
        num = num + 1;
        drive_index++;
        free(wheel_obj);
    }
}

void v_wheel_limited(int N_wh, v_wheel * vwheels) {
    int state = 0;
    double max_v_wheel_radio = 1;
    int i;

    for (i = 0; i < N_wh; i++) {
        if (vwheels[i]._w_type != kOnlySWheel) {
            if (fabs(vwheels[i]._vo) > vwheels[i].max_speed) {
                state = 1;
                double v_radio = fabs(vwheels[i]._vo) / vwheels[i].max_speed;
                if (v_radio > max_v_wheel_radio) {
                    max_v_wheel_radio = v_radio;
                }
            }
        }
    }
    if (state == 1) {
        for (i = 0; i < N_wh; i++) {
            if (vwheels[i]._w_type != kOnlySWheel) {
                vwheels[i]._vo = vwheels[i]._vo / max_v_wheel_radio;
            }
        }
    }
}

double Fscore(int N_wh, v_wheel* v_wh, double* X) {
    double dt = 0.1;
    double sigma_delta_roll2 = 0;
    double this_delta_roll2;
    double sigma_delta_slide2 = 0;
    double this_delta_slide2;
    int i=0;

    for (i = 0; i < N_wh; i++) {
        double thisprojv, thisv_need;
        thisprojv = v_wh[i]._vp;
        thisv_need = X[i];
        this_delta_roll2 = v_wh[i]._roll_wt * (thisprojv - thisv_need)*(thisprojv - thisv_need) * dt*dt;
        sigma_delta_roll2 += this_delta_roll2;
    }
    for (i = 0; i < N_wh; i++) {
        double thisv_need;
        thisv_need = X[i];
        double x_next, y_next;
        x_next = v_wh[i]._x + (thisv_need * dt) * cos(v_wh[i]._af);
        y_next = v_wh[i]._y + (thisv_need * dt) * sin(v_wh[i]._af);
        double xc_need, yc_need, ac_need;
        xc_need = X[N_wh];
        yc_need = X[N_wh + 1];
        ac_need = X[N_wh + 2];
        double x_need, y_need;
        x_need = v_wh[i]._x * cos(ac_need) - v_wh[i]._y * sin(ac_need) + xc_need;
        y_need = v_wh[i]._y * sin(ac_need) + v_wh[i]._y * cos(ac_need) + yc_need;
        this_delta_slide2 = v_wh[i]._slide_wt * ((x_need - x_next)*(x_need - x_next) + (y_need - y_next)*(y_need - y_next));
        sigma_delta_slide2 += this_delta_slide2;
    }
    double sigma_detla;
    sigma_detla = sigma_delta_roll2 + sigma_delta_slide2;
    return sigma_detla;
}

void DoOptimize(double *Xresult, int N_wh, v_wheel* vwheels, double* Xinit, char *pval) {
    *pval = 0;
    int N_wh1 = N_wh + 3;
    int i, u;
    double delta, alfa, beta, uplon;
    delta = 0.01;
    alfa = 1.1;
    beta = 0.95;
    uplon = 1e-4;

    double *pos_now = (double*) malloc(sizeof ( double)*N_wh1);
    for (i = 0; i < N_wh1; i++) {
        *(pos_now + i) = *(Xinit + i);
    }


    double *pos_last = (double*) malloc(sizeof ( double)*N_wh1);
    double *E_axis_temp = (double*) malloc(sizeof ( double)*N_wh1);
    double *pos_end = (double*) malloc(sizeof ( double)*N_wh1);
    memset(pos_last, 0, sizeof ( double)*N_wh1);
    memset(E_axis_temp, 0, sizeof ( double)*N_wh1);
    memset(pos_end, 0, sizeof ( double)*N_wh1);


    double **spos_axis;
    spos_axis = (double**) malloc(sizeof ( double*)*(N_wh1 + 1));
    for (i = 0; i < N_wh1 + 1; i++) {
        spos_axis[i] = (double*) malloc(sizeof ( double)*N_wh1);
        memset(spos_axis[i], 0, sizeof ( double)*N_wh1);
    }

    double **E_axis;
    E_axis = (double**) malloc(sizeof ( double*)*N_wh1);
    for (i = 0; i < N_wh1; i++) {
        E_axis[i] = (double*) malloc(sizeof ( double)*N_wh1);
        memset(E_axis[i], 0, sizeof ( double)*N_wh1);
    }

    double *posaxis_inc = (double*) malloc(sizeof ( double)*N_wh1);
    memset(posaxis_inc, 0, sizeof ( double)*N_wh1);
    double *posaxis_dec = (double*) malloc(sizeof ( double)*N_wh1);
    memset(posaxis_dec, 0, sizeof ( double)*N_wh1);

	for (i = 0; i < N_wh1; i++) {
		*(pos_last + i) = *(pos_now + i);
	}
    double *datas;
    datas = (double *) malloc(sizeof ( double)* N_wh1); 

    int v;
    for (u = 0; u < N_wh1; u++) {
        for (v = 0; v < N_wh1; v++) {
            datas[v] = 0;
        }
        datas[u] = 1;
        memcpy(E_axis[u], datas, sizeof ( double)* N_wh1);
    }
    free(datas);

    int k = 0;
    int j = 0;
    int num_cal = 0; 
    char stop = 0;
    for (i = 0; i < N_wh1; i++) {
        *(spos_axis[0] + i) = *(pos_now + i);
    }
    double prob_now = 0;

    while (1) {
        if (stop) break;
        if (k > 3000) {
            break;
        }
        prob_now = Fscore(N_wh, vwheels, pos_now);
        num_cal++;

        while (1) {
            for (i = 0; i < N_wh1; i++) {
                *(E_axis_temp + i) = *(E_axis[j] + i) * delta;
            }
            for (i = 0; i < N_wh1; i++) {
                *(posaxis_inc + i) = *(spos_axis[j] + i) + *(E_axis_temp + i);
            }	
            for (i = 0; i < N_wh1; i++) {
                *(posaxis_dec + i) = *(spos_axis[j] + i) - *(E_axis_temp + i);
            }
            if (Fscore(N_wh, vwheels, posaxis_inc) < prob_now) {
                memcpy(spos_axis[j + 1], posaxis_inc, sizeof ( double)* N_wh1);
                num_cal++;
            } else {
                if (Fscore(N_wh, vwheels, posaxis_dec) < prob_now) {
                    memcpy(spos_axis[j + 1], posaxis_dec, sizeof ( double)* N_wh1);
                    num_cal++;
                } else
                    memcpy(spos_axis[j + 1], spos_axis[j], sizeof ( double)* N_wh1);
            }

            if (j < N_wh1 - 1) {
                j++;
            } else {
                break;
            }
        }

        if (Fscore(N_wh, vwheels, spos_axis[N_wh1]) < prob_now)
        {
            for (i = 0; i < N_wh1; i++) {
                *(pos_last + i) = *(pos_now + i);
            }
            for (i = 0; i < N_wh1; i++) {
                *(pos_now + i) = *(spos_axis[N_wh1] + i);
            }
            for (i = 0; i < N_wh1; i++) {
                *(spos_axis[0] + i) = *(pos_now + i) + (*(pos_now + i) - *(pos_last + i)) * alfa;
            }
            j = 0;
            k++;
            num_cal++;
        } else {
            if (delta < uplon) {
                stop = 1;
                for (i = 0; i < N_wh1; i++) {
                    *(pos_end + i) = *(pos_now + i);
                }
            } else {
                delta = beta*delta;
                for (i = 0; i < N_wh1; i++) {
                    *(spos_axis[0] + i) = *(pos_now + i);
                }
                for (i = 0; i < N_wh1; i++) {
                    *(pos_last + i) = *(pos_now + i);
                }
                j = 0;
                k++;
            }
        }

    }

    if (k < 3000) {
        *pval = 1;
        for (i = 0; i < N_wh1; i++) {
            *(Xresult + i) = *(pos_end + i);
        }
    } else {
        *pval = 0;
    }
    free(pos_now);
    free(pos_last);
    free(E_axis_temp);
    free(pos_end);
    for (i = 0; i < N_wh1 + 1; i++) {
        free(spos_axis[i]);
    }
    free(spos_axis);
    for (i = 0; i < N_wh1; i++) {
        free(E_axis[i]);
    }
    free(E_axis);
    free(posaxis_inc);
    free(posaxis_dec);
}

int GetOpti_va(int N_wh, v_wheel *vw, double* va_init) {
    int i=0;
    for (i = 0; i < N_wh; i++) {
        vw[i]._vo = vw[i]._vp;
    }
    return 1;
}

void WriteToGlobal(int N_wh, v_wheel * vwheels) {
    int i=0;
    for (i = 0; i < N_wh; i++) {
        if (vwheels[i]._w_type == kOnlyDWheel) {
            var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, vwheels[i]._dwheel_1_obj);
            dwheel->i.command_velocity_enc_ = (int64_t)(vwheels[i]._vo * dwheel->scale_control_);
            var__commit_wheel_dup(vwheels[i]._dwheel_1_obj);
            var__release_wheel_dup(vwheels[i]._dwheel_1_obj);
        } else if (vwheels[i]._w_type == kOnlySWheel) {
            var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, vwheels[i]._swheel_obj);
			swheel->i.command_angle_enc_ = (int64_t)((vwheels[i]._ae - vwheels[i]._zero_angle)* swheel->scale_control_);
            var__commit_wheel_dup(vwheels[i]._swheel_obj);
            var__release_wheel_dup(vwheels[i]._swheel_obj);
        } else if (vwheels[i]._w_type == kSDWheel) {
            var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, vwheels[i]._swheel_obj);
			var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, vwheels[i]._dwheel_1_obj);
			swheel->i.command_angle_enc_ = (int64_t)((vwheels[i]._ae - vwheels[i]._zero_angle)* swheel->scale_control_);
            dwheel->i.command_velocity_enc_ = (int64_t)(vwheels[i]._vo * dwheel->scale_control_); 
			var__commit_wheel_dup(vwheels[i]._swheel_obj);
			var__release_wheel_dup(vwheels[i]._swheel_obj);
            var__commit_wheel_dup(vwheels[i]._dwheel_1_obj);
            var__release_wheel_dup(vwheels[i]._dwheel_1_obj);
        } else {
            if (vwheels[i].left_or_right) {
                var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, vwheels[i]._dwheel_1_obj);
                dwheel->i.command_velocity_enc_ = (int64_t)(vwheels[i]._vo * dwheel->scale_control_);
                var__commit_wheel_dup(vwheels[i]._dwheel_1_obj);
                var__release_wheel_dup(vwheels[i]._dwheel_1_obj);
                if (vwheels[i]._swheel_obj)
                {
                    var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, vwheels[i]._swheel_obj);
                    swheel->i.command_angle_enc_ = (int64_t)((vwheels[i]._ae - vwheels[i]._zero_angle)* swheel->scale_control_);
                    var__commit_wheel_dup(vwheels[i]._swheel_obj);
                    var__release_wheel_dup(vwheels[i]._swheel_obj);
                }
            } else {
                var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, vwheels[i]._dwheel_2_obj);
                dwheel->i.command_velocity_enc_ = (int64_t)(vwheels[i]._vo * dwheel->scale_control_);
                var__commit_wheel_dup(vwheels[i]._dwheel_2_obj);
                var__release_wheel_dup(vwheels[i]._dwheel_2_obj);
            }
        }
    }
}


int VwResolution(v_chassis *V_com, var__vehicle_t *veh, const struct list_head *drive_unit) {
    int i;
    int state = 1;
    int num_time_stamp;
    int num_wh = get_num_of_wheel(drive_unit, &num_time_stamp);
    v_wheel * v_wheels = (v_wheel*) malloc(sizeof (v_wheel) * num_wh);
    if (sdd_firsttime == 0) {
        w_sdd_last = (double*) malloc(sizeof (double)*num_wh);
        memset(w_sdd_last, 0, sizeof (double)*num_wh);
    }
    first_v_decomposite(v_wheels, drive_unit, veh);
    double * va_init = (double*) malloc(sizeof ( double)*(num_wh + 3));
    for (i = 0; i < num_wh + 3; i++) {
        if (i < num_wh)
            va_init[i] = v_wheels[i]._vp;
        else
            va_init[i] = 0;
    }
    GetOpti_va(num_wh, v_wheels, va_init);
    v_wheel_limited(num_wh, v_wheels); 
    if (v_composition(v_wheels, num_wh, 0, &(V_com[0])) == 0) {
        state = 0;
    }
    if (v_composition(v_wheels, num_wh, 1, &(V_com[1])) == 0) {
        state = 0;
    }
#if DATA_W
	data_w_log(v_wheels, num_wh);
#endif
    WriteToGlobal(num_wh, v_wheels);
    free(v_wheels);	
    free(va_init);
    return state;
}

double trim_solo_only_acc(double v_x, double v_x_last, double v_acc) {
    double result = 0;
    if (v_x * v_x_last >= 0) {
        if ((v_x >= 0) && (v_x_last >= 0)) {
            if ((v_x - v_x_last) > v_acc) {
                result = v_x_last + v_acc;
            } else {
                result = v_x;
            }
        }
        if ((v_x <= 0) && (v_x_last <= 0)) {
            if ((v_x - v_x_last) < -v_acc) {
                result = v_x_last - v_acc;
            } else {
                result = v_x;
            }
        }
    } else {
        if ((v_x - v_x_last) > v_acc) {
            result = v_x_last + v_acc;
        } else {
            if ((v_x - v_x_last) < -v_acc) {
                result = v_x_last - v_acc;
            } else {
                result = v_x;
            }
        }
    }
    return result;
}

double trim_solo(double v_x, double v_x_last, double v_dec, double v_acc) {
    double result = 0;
    if (v_x * v_x_last >= 0) {
        if ((v_x >= 0) && (v_x_last >= 0)) {
            if ((v_x - v_x_last) > v_acc) {
                result = v_x_last + v_acc;
            } else {
                if ((v_x - v_x_last) < -v_dec) {
                    result = v_x_last - v_dec;
                } else {
                    result = v_x;
                }
            }
        }
        if ((v_x <= 0) && (v_x_last <= 0)) {
            if ((v_x - v_x_last) > v_dec) {
                result = v_x_last + v_dec;
            } else {
                if ((v_x - v_x_last) < -v_acc) {
                    result = v_x_last - v_acc;
                } else {
                    result = v_x;
                }
            }
        }
    } else {
        double min_acc_dec = min(v_acc, v_dec);
        if ((v_x - v_x_last) > min_acc_dec) {
            result = v_x_last + min_acc_dec;
        } else {
            if ((v_x - v_x_last) < -min_acc_dec) {
                result = v_x_last - min_acc_dec;
            } else {
                result = v_x;
            }
        }
    }
    return result;
}

void trim(double v_x, double v_y, double w, var__navigation_t *nav, var__vehicle_t *veh, double deta_time, int trim_type, velocity_t *result) {
    double v_x_last = veh->i.command_velocity_.x_;
    double v_y_last = veh->i.command_velocity_.y_;
    double w_last = veh->i.command_velocity_.w_;
    double v_dec_stop = nav->dec_estop_*deta_time;

    double v_dec=0, w_dec=0, v_acc=0, w_acc=0, v_max=0, w_max=0;
    if (trim_type == VEHICLE_LIMIT) {
        v_dec = veh->max_dec_*deta_time;
        w_dec = veh->max_dec_w_*deta_time;
        v_acc = veh->max_acc_*deta_time;
        w_acc = veh->max_acc_w_*deta_time;

        v_max = veh->max_speed_;
        w_max = veh->max_w_;
    } else {
        v_dec = nav->dec_*deta_time;
        w_dec = nav->dec_w_*deta_time;
        v_acc = nav->acc_*deta_time;
        w_acc = nav->acc_w_*deta_time;

        v_max = nav->max_speed_;
        w_max = nav->max_w_;
    }
    if (trim_type == ONLY_ACC) {
        result->x_ = trim_solo_only_acc(v_x, v_x_last, v_acc);
        result->y_ = trim_solo_only_acc(v_y, v_y_last, v_acc);
        result->w_ = trim_solo_only_acc(w, w_last, w_acc);
        result->x_ = (result->x_ >= 0 ? 1 : -1) * min(fabs(result->x_), v_max);
        result->y_ = (result->y_ >= 0 ? 1 : -1) * min(fabs(result->y_), v_max);
        result->w_ = (result->w_ >= 0 ? 1 : -1) * min(fabs(result->w_), w_max);
    } else {
        if (trim_type == DEC_DSTOP) {
            result->x_ = trim_solo(v_x, v_x_last, v_dec_stop, v_acc);
            result->y_ = trim_solo(v_y, v_y_last, v_dec_stop, v_acc);
        } else {
            result->x_ = trim_solo(v_x, v_x_last, v_dec, v_acc);
            result->y_ = trim_solo(v_y, v_y_last, v_dec, v_acc);
        }
        result->w_ = trim_solo(w, w_last, w_dec, w_acc);
        result->x_ = (result->x_ >= 0 ? 1 : -1) * min(fabs(result->x_), v_max);
        result->y_ = (result->y_ >= 0 ? 1 : -1) * min(fabs(result->y_), v_max);
        result->w_ = (result->w_ >= 0 ? 1 : -1) * min(fabs(result->w_), w_max);
    }
}

void trim2(double v, var__navigation_t *nav, var__vehicle_t *veh, double deta_time, int trim_type, double *result) {
    double v_x_last = veh->i.command_velocity_.x_;
    double v_y_last = veh->i.command_velocity_.y_;
    double v_last = sqrt(v_x_last * v_x_last + v_y_last * v_y_last);

    double v_acc=0, v_max=0, v_dec=0;
    v_acc = nav->acc_*deta_time;
    v_max = nav->max_speed_;
    v_dec = nav->dec_*deta_time;
    if (trim_type == ONLY_ACC) {
        *result = trim_solo_only_acc(v, v_last, v_acc);
    }
    if (trim_type == NON) {
        *result = trim_solo(v, v_last, v_dec, v_acc);
    }
    *result = (*result >= 0 ? 1 : -1) * min(fabs(*result), v_max);
}

int InvertGaussJordan(double *m_pData, int m_nNumColumns)
{
    int *pnRow, *pnCol, i, j, k, l, u, v;
    double d = 0, p = 0;

    pnRow = (int*) malloc(sizeof ( int)*m_nNumColumns);
    pnCol = (int*) malloc(sizeof ( int)*m_nNumColumns);
    if (pnRow == NULL || pnCol == NULL)
        return 0;

    for (k = 0; k <= m_nNumColumns - 1; k++) {
        d = 0.0;
        for (i = k; i <= m_nNumColumns - 1; i++) {
            for (j = k; j <= m_nNumColumns - 1; j++) {
                l = i * m_nNumColumns + j;
                p = fabs(m_pData[l]);
                if (p > d) {
                    d = p;
                    pnRow[k] = i;
                    pnCol[k] = j;
                }
            }
        }

        if (d == 0.0) {
            free(pnRow);
            free(pnCol);
            return 0;
        }

        if (pnRow[k] != k) {
            for (j = 0; j <= m_nNumColumns - 1; j++) {
                u = k * m_nNumColumns + j;
                v = pnRow[k] * m_nNumColumns + j;
                p = m_pData[u];
                m_pData[u] = m_pData[v];
                m_pData[v] = p;
            }
        }

        if (pnCol[k] != k) {
            for (i = 0; i <= m_nNumColumns - 1; i++) {
                u = i * m_nNumColumns + k;
                v = i * m_nNumColumns + pnCol[k];
                p = m_pData[u];
                m_pData[u] = m_pData[v];
                m_pData[v] = p;
            }
        }

        l = k * m_nNumColumns + k;
        m_pData[l] = 1.0 / m_pData[l];
        for (j = 0; j <= m_nNumColumns - 1; j++) {
            if (j != k) {
                u = k * m_nNumColumns + j;
                m_pData[u] = m_pData[u] * m_pData[l];
            }
        }

        for (i = 0; i <= m_nNumColumns - 1; i++) {
            if (i != k) {
                for (j = 0; j <= m_nNumColumns - 1; j++) {
                    if (j != k) {
                        u = i * m_nNumColumns + j;
                        m_pData[u] = m_pData[u] - m_pData[i * m_nNumColumns + k] * m_pData[k * m_nNumColumns + j];
                    }
                }
            }
        }

        for (i = 0; i <= m_nNumColumns - 1; i++) {
            if (i != k) {
                u = i * m_nNumColumns + k;
                m_pData[u] = -m_pData[u] * m_pData[l];
            }
        }
    }

    for (k = m_nNumColumns - 1; k >= 0; k--) {
        if (pnCol[k] != k) {
            for (j = 0; j <= m_nNumColumns - 1; j++) {
                u = k * m_nNumColumns + j;
                v = pnCol[k] * m_nNumColumns + j;
                p = m_pData[u];
                m_pData[u] = m_pData[v];
                m_pData[v] = p;
            }
        }

        if (pnRow[k] != k) {
            for (i = 0; i <= m_nNumColumns - 1; i++) {
                u = i * m_nNumColumns + k;
                v = i * m_nNumColumns + pnRow[k];
                p = m_pData[u];
                m_pData[u] = m_pData[v];
                m_pData[v] = p;
            }
        }
    }

    free(pnRow);
    free(pnCol);

    return 1;
}

int v_composition(v_wheel * v_wheels, int num_wheel, int ref_enc, v_chassis *out)
{
    uint64_t sum_time = 0;
    int num_v = num_wheel;
    double *A = (double*) malloc(sizeof (double)* 9);
    memset(A, 0, sizeof (double)* 9);
    double *B = (double*) malloc(sizeof (double)* 3);
    memset(B, 0, sizeof (double)* 3);
    int i=0;
    int sdd_index = -1;
    double vl_wheel=0, vr_wheel=0, xl=0, yl=0, xr=0, yr=0; 
    for (i = 0; i < num_wheel; i++) {
        double angle=0, x=0, y=0, v=0, r=0, s=0;
        if (v_wheels[i]._w_type == kSDWheel) {
            x = v_wheels[i]._x;
            y = v_wheels[i]._y;
            if (ref_enc == 1) {
                angle = v_wheels[i]._af;
                var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, v_wheels[i]._swheel_obj);
                sum_time = swheel->time_stamp_ + sum_time;
                num_v = num_v + 1;
                var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, v_wheels[i]._dwheel_1_obj);
                v = dwheel->actual_velocity_enc_ / dwheel->scale_feedback_;
                sum_time = dwheel->time_stamp_ + sum_time;
            } else {
                angle = v_wheels[i]._ae;
                v = v_wheels[i]._vo;
            }
            r = v_wheels[i]._roll_wt;
            s = v_wheels[i]._slide_wt;
        } else if (v_wheels[i]._w_type == kOnlyDWheel) {
            x = v_wheels[i]._x;
            y = v_wheels[i]._y;
            angle = v_wheels[i]._af;
            if (ref_enc == 1) {
                var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, v_wheels[i]._dwheel_1_obj);
                v = dwheel->actual_velocity_enc_ / dwheel->scale_feedback_;
                sum_time = dwheel->time_stamp_ + sum_time;
            } else {
                v = v_wheels[i]._vo;
            }
            r = v_wheels[i]._roll_wt;
            s = v_wheels[i]._slide_wt;
        } else if (v_wheels[i]._w_type == kOnlySWheel) {
            x = v_wheels[i]._x;
            y = v_wheels[i]._y;
            if (ref_enc == 1) {
                var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, v_wheels[i]._swheel_obj);
                sum_time = swheel->time_stamp_ + sum_time;
                angle = v_wheels[i]._af;
            } else
                angle = v_wheels[i]._ae;
            r = 0;
            s = 0;
            v = 0;
        } else {
            if (v_wheels[i].index_drive != sdd_index && v_wheels[i].left_or_right == 1) {
                sdd_index = v_wheels[i].index_drive;
                xl = v_wheels[i]._x;
                yl = v_wheels[i]._y;
                if (ref_enc == 1) {
                    var__swheel_t *swheel = var__object_body_ptr(var__swheel_t, v_wheels[i]._swheel_obj);
                    sum_time = swheel->time_stamp_ + sum_time;
                    num_v = num_v + 1;
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, v_wheels[i]._dwheel_1_obj);
                    vl_wheel = dwheel->actual_velocity_enc_ / dwheel->scale_feedback_;
                    sum_time = dwheel->time_stamp_ + sum_time;
                } else
                    vl_wheel = v_wheels[i]._vo;
                i = i + 1;
            }
            if (v_wheels[i].index_drive == sdd_index && v_wheels[i].left_or_right == 0) {
                xr = v_wheels[i]._x;
                yr = v_wheels[i]._y;
                if (ref_enc == 1) {
                    var__dwheel_t *dwheel = var__object_body_ptr(var__dwheel_t, v_wheels[i]._dwheel_2_obj);
                    vr_wheel = dwheel->actual_velocity_enc_ / dwheel->scale_feedback_;
                    sum_time = dwheel->time_stamp_ + sum_time;
                } else
                    vr_wheel = v_wheels[i]._vo;
                v = (vl_wheel + vr_wheel) / 2.0;
                x = (xl + xr) / 2.0;
                y = (yl + yr) / 2.0;
                angle = v_wheels[i]._af;
                r = v_wheels[i]._roll_wt;
                s = v_wheels[i]._slide_wt;
                sdd_index = -1;
            }
        }
        double sita = atan2(y, x);
        double f1 = -sqrt(x * x + y * y) * sin(sita);
        double f2 = sqrt(x * x + y * y) * cos(sita);
        A[0] = A[0] + r * r * f1 * f1 + s * s * f2*f2;
        A[1] = A[1] + r * r*f1;
        A[2] = A[2] + s * s*f2;
        A[4] = A[4] + r*r;
        A[5] = 0;
        A[8] = A[8] + s*s;
        B[0] = B[0] + (r * r * f1 * cos(angle) + s * s * f2 * sin(angle)) * v;
        B[1] = B[1] + r * r * cos(angle) * v;
        B[2] = B[2] + s * s * sin(angle) * v;
    }
    A[3] = A[1];
    A[6] = A[2];
    A[7] = A[5];
   if (InvertGaussJordan(A, 3) == 0) {
        out->time_stamp = sum_time / (uint64_t) num_v;
        free(A);
        free(B);
        return 0;
    }
    out->W = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
    out->V_x = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
    out->V_y = A[6] * B[0] + A[7] * B[1] + A[8] * B[2];
    out->time_stamp = sum_time / (uint64_t) num_v;
    free(A);
    free(B);
    return 1;
}

#if DATA_W
void data_w_log(v_wheel * v_wheels, int num_wheel)
{
	int i = 0;
	char str[260] = { 0 };
	double v_act = 0;
	unsigned long time_stamp = 0;
	int sdd_index = -1;
	var__dwheel_t *dwheel;
	var__swheel_t *swheel;
	for (i=0; i < num_wheel; i++)
	{
		char str_temp[20] = { 0 };
		switch (v_wheels[i]._w_type)
		{
		case kOnlyDWheel:
			memset(str_temp, 0, sizeof (char)* 20);
			_itoa_s(v_wheels[i]._w_type, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_itoa_s(v_wheels[i].index_drive, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_gcvt_s(str_temp, 20, v_wheels[i]._vo, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			dwheel = var__object_body_ptr(var__dwheel_t, v_wheels[i]._dwheel_1_obj);
			v_act = dwheel->actual_velocity_enc_ / dwheel->scale_feedback_;
			memset(str_temp, 0, sizeof (char)* 20);
			_gcvt_s(str_temp, 20, v_act, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			time_stamp = (unsigned long)dwheel->time_stamp_;
			memset(str_temp, 0, sizeof (char)* 20);
			_ultoa_s(time_stamp, str_temp,20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);
			break;
		case kOnlySWheel:
			memset(str_temp, 0, sizeof (char)* 20);
			_itoa_s(v_wheels[i]._w_type, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_itoa_s(v_wheels[i].index_drive, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_gcvt_s(str_temp, 20, v_wheels[i]._ae, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_gcvt_s(str_temp, 20, v_wheels[i]._af, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			swheel = var__object_body_ptr(var__swheel_t, v_wheels[i]._swheel_obj);
			time_stamp = (unsigned long)swheel->time_stamp_;
			memset(str_temp, 0, sizeof (char)* 20);
			_ultoa_s(time_stamp, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);
			break;
		case kSDWheel:
			memset(str_temp, 0, sizeof (char)* 20);
			_itoa_s(v_wheels[i]._w_type, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_itoa_s(v_wheels[i].index_drive, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_gcvt_s(str_temp, 20, v_wheels[i]._vo, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			dwheel = var__object_body_ptr(var__dwheel_t, v_wheels[i]._dwheel_1_obj);
			v_act = dwheel->actual_velocity_enc_ / dwheel->scale_feedback_;
			memset(str_temp, 0, sizeof (char)* 20);
			_gcvt_s(str_temp, 20, v_act, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			time_stamp = (unsigned long)dwheel->time_stamp_;
			memset(str_temp, 0, sizeof (char)* 20);
			_ultoa_s(time_stamp, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_gcvt_s(str_temp, 20, v_wheels[i]._ae, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			memset(str_temp, 0, sizeof (char)* 20);
			_gcvt_s(str_temp, 20, v_wheels[i]._af, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);

			swheel = var__object_body_ptr(var__swheel_t, v_wheels[i]._swheel_obj);
			time_stamp = (unsigned long)swheel->time_stamp_;
			memset(str_temp, 0, sizeof (char)* 20);
			_ultoa_s(time_stamp, str_temp, 20, 10);
			posix__strcat(str, sizeof(str), " ");
			posix__strcat(str, sizeof(str), str_temp);
			break;
		case kSDDWheel:
			if (v_wheels[i].index_drive != sdd_index && v_wheels[i].left_or_right == 1)
			{
				sdd_index = v_wheels[i].index_drive;
				memset(str_temp, 0, sizeof (char)* 20);
				_itoa_s(v_wheels[i]._w_type, str_temp, 20, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				memset(str_temp, 0, sizeof (char)* 20);
				_itoa_s(v_wheels[i].index_drive, str_temp, 20, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				memset(str_temp, 0, sizeof (char)* 20);
				_gcvt_s(str_temp, 20, v_wheels[i]._vo, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				dwheel = var__object_body_ptr(var__dwheel_t, v_wheels[i]._dwheel_1_obj);
				v_act = dwheel->actual_velocity_enc_ / dwheel->scale_feedback_;
				memset(str_temp, 0, sizeof (char)* 20);
				_gcvt_s(str_temp, 20, v_act, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				time_stamp = (unsigned long)dwheel->time_stamp_;
				memset(str_temp, 0, sizeof (char)* 20);
				_ultoa_s(time_stamp, str_temp, 20, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				memset(str_temp, 0, sizeof (char)* 20);
				_gcvt_s(str_temp, 20, v_wheels[i]._ae, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				memset(str_temp, 0, sizeof (char)* 20);
				_gcvt_s(str_temp, 20, v_wheels[i]._af, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				swheel = var__object_body_ptr(var__swheel_t, v_wheels[i]._swheel_obj);
				time_stamp = (unsigned long)swheel->time_stamp_;
				memset(str_temp, 0, sizeof (char)* 20);
				_ultoa_s(time_stamp, str_temp, 20, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);
			}
			if (v_wheels[i].index_drive == sdd_index && v_wheels[i].left_or_right == 0)
			{
				memset(str_temp, 0, sizeof (char)* 20);
				_gcvt_s(str_temp, 20, v_wheels[i]._vo, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				dwheel = var__object_body_ptr(var__dwheel_t, v_wheels[i]._dwheel_2_obj);
				v_act = dwheel->actual_velocity_enc_ / dwheel->scale_feedback_;
				memset(str_temp, 0, sizeof (char)* 20);
				_gcvt_s(str_temp, 20, v_act, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);

				time_stamp = (unsigned long)dwheel->time_stamp_;
				memset(str_temp, 0, sizeof (char)* 20);
				_ultoa_s(time_stamp, str_temp, 20, 10);
				posix__strcat(str, sizeof(str), " ");
				posix__strcat(str, sizeof(str), str_temp);
			}
			break;
		}
	}
	log__save("data_w", kLogLevel_Info, kLogTarget_Filesystem, "%s", str);
}
#endif