//
//  Copyright (C) 2016-2022  Vuelosophy, Inc. - All Rights Reserved
//
// Eye Tracking Pubic API
//
#ifndef __PUBH__
#define __PUBH__

#include <stdint.h>

#define DFT_ET_IDX              0
#define UNK_FOCLEN              0
#define DFT_FT_FOCLEN           800
#define DFT_ET0_FOCLEN          640
#define CALI_SAMPLE_TOTAL       3

enum GAZE_META
{
    GAZE_META_ET_IDX = 0,       //eye camera index, can be 0, 1
    GAZE_META_ET_NUM,           //eye frame number
    GAZE_META_HAS_PUPIL,        //has valid pupil data
    GAZE_META_PUPIL_X ,         //pupil position x in frame in unit of pixel
    GAZE_META_PUPIL_Y,          //pupil position y in frame in unit of pixel
    GAZE_META_FIX,              //in fixation, can be 0, 1
    GAZE_META_TOTAL
};

/**
 * @brief  eye tracking 2D gaze point callback
 * @param  gaze[o], gaze[1] : gaze point
 */
typedef void (*PUB_GAZE_2D_CB)( float *gaze, int64_t ts, void *meta, void *refcon );

struct PUB;

//*** to convert legacy pub_set_conf_gaze to pub_set_conf_gaze_o
/*
#define pub_set_conf_gaze( pub, et_idx, s_idx, conf_gaze_x, conf_gaze_y )   \
do {                                                                        \
    float conf_gaze_o[2] = { conf_gaze_x, conf_gaze_y };                    \
    pub_set_conf_gaze_o( vue->pub, et_idx, vue->s_idx, conf_gaze_o );       \
}while(0)
*/

extern "C" {

/**
 * @brief  get gaze 3D vector, unitized, from gaze 2D point on screen
 * @param   gaze_v : gaze 3D vector
 *          gaze_o : gaze 2D point on screen
 */
void pub_v0_frm_o(struct PUB *pub, float *gaze_v, float *gaze_o);

/**
 * @brief  set confirmed gaze 3D vector
 * @param   et_idx : eye index
 *          s_idx : gaze point index can be 0, 1, 2
 *          conf_gaze_v: gaze vector screen 3D coordinate system, [0] x, [1] y, [2] z
 * @return  1 succeed, 0 failure
 */
int pub_set_conf_gaze_v(struct PUB *pub, int et_idx, int s_idx, float *conf_gaze_v);

/**
 * @brief  set confirmed gaze point on screen
 * @param   et_idx : eye index
 *          s_idx : gaze point on screen, index can be 0, 1, 2
 *          conf_gaze_o: gaze point on screen in unit of pixel, [0] x, [1] y
 * @return  1 succeed, 0 failure
 */
int pub_set_conf_gaze_o(struct PUB *pub, int et_idx, int s_idx, float *conf_gaze_o);

/**
 * @brief  input eye tracking image
 * @param   et_idx : eye index
 *          img : gray scale eye camera image
 *          size : frame size in unit of bytes
 *          ts : time stamp
 * @return  1 succeed, 0 failure
 */
void pub_push_et_img(struct PUB *pub, int et_idx, uint8_t *img, int size, int64_t ts, int num);

/**
 * @brief   set eye tracking data callback, gaze point on screen in unit of pixel
 * @param   et_idx : eye index
 *          img : gray scale eye camera image
 *          size : frame size in unit of bytes
 *          ts : time stamp
 */
void pub_set_gaze_o_cb(struct PUB *pub, PUB_GAZE_2D_CB gaze_o_cb, void *gaze_o_refcon);

/**
 * @brief   3D gaze vector from  gaze point on screen
 * @param   gaze_v : unitized 3D gaze vector in screen coordinate system
 *          gaze_o: gaze point on screen in unit of pixel
 */
void pub_v0_frm_o(struct PUB *pub, float *gaze_v, float *gaze_o);

/**
 * @brief   get eye center 3D position
 * @param   et_idx : eye index
 *          oe_v: eye center 3D position in et camera coordinate system, in unit of eye radius
 * @return  1 succeed, 0 failure
 */
int pub_get_oe_v(struct PUB *pub, int et_idx, float *oe_v);

/**
 * @brief   set preference file path
 * @param   et_idx : eye index
 *          path : preference file path
 */
void pub_set_pref_path(struct PUB *pub, int et_idx, char *path);

/**
 * @brief   set camera matrix and distortion coefficients
 * @param   cam_m : camera matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
 *          disco : distortion coefficients: [k1, k2, p1, p2, k3]
 */
void pub_set_cam_m_disco(struct PUB *pub, float *cam_m, float *disco);

/**
 * @brief  set slippage resistence value
 * @param   sr_val : 0 off, 1 midium, 2 high
*/
void pub_set_sr_val(struct PUB *pub, int sr_val);

/**
 * @brief  reset internal state
 */
void reset_pub(struct PUB *pub);

/**
 * @brief  dispose instance
 */
void dispose_pub(struct PUB *pub);

/**
 * @brief  create new eye tracking instance
 * @param   ft_wid, ft_hgt : screen dimension in unit of pixel
 *          ft_foclen      : screen focal length in unit of pixel
 *          et_wid, et_hgt : eye camera frame dimension in unit of pixel
 *          et_foclen      : eye camera focal length in unit of pixel
 *          et_fps         : eye camera frame rate
 *          dir            : directory of data files, it needs to end with "/", NULL is using .so dir
 */
struct PUB *new_pub( int ft_wid, int ft_hgt, float ft_foclen, int et_wid, int et_hgt, float et_foclen, float et_fps, int et_tot );

/**
 * @brief  initialize environment
 * @param  dir            : directory of data files, it needs to end with "/", NULL is using .so dir
 * @return  1 succeed, 0 failure
 */
int init_vue( char *dir );

}
#endif  //__PUBH__
