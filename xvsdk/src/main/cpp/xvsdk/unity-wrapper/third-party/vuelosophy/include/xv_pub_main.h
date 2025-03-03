//
//  Copyright (C) 2016-2022  Vuelosophy, Inc. - All Rights Reserved
//
// Eye Tracking Pubic API
//
#ifndef __XV_PUBH_MAIN___
#define __XV_PUBH_MAIN___

typedef void (*xv_pub_callback)(int etIndex, float gaze_x, float gaze_y, float pupl_x, float pupl_y, int64_t ts);
typedef void (*xv_pub_dbg_callback)(int etIndex, void *data, int total);

extern "C" {
void xv_pub_init(int et_total, int screen_width, int screen_height,
                 int cam_width, int cam_height, int cam_fps);

void xv_pub_register(xv_pub_callback callback);

void xv_pub_register_dbg(xv_pub_dbg_callback callback);

void xv_pub_set_img(uint8_t *left_img, uint8_t *right_img, int size,
                    int64_t ts);

int xv_pub_set_calibration(int eye, int index, float gaze_x, float gaze_y);

void xv_pub_dispose();

void xv_pub_set_asset_path(const char* path);
}
#endif //__XV_PUBH_MAIN___
