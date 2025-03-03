//
//  Copyright (C) 2016-2022  Vuelosophy, Inc. - All Rights Reserved
//
// Eye Tracking Pubic DEBUG API
//
#ifndef __PUBDBGH__
#define __PUBDBGH__

#include <stdint.h>

extern "C" {

enum DBG_CUSTOM_META
{
    DBG_META_ET_IDX = 0,        //eye camera index, can be 0, 1
    DBG_META_ET_NUM,            //eye frame number
    DBG_META_HAS_PUPIL,         //has valid pupil data
    DBG_META_PUPIL_X,           //pupil position x in frame in unit of pixel
    DBG_META_PUPIL_Y,           //pupil position y in frame in unit of pixel
    DBG_META_PUPIL_BOX_LFT ,    //pupil box position top  in frame in unit of pixel
    DBG_META_PUPIL_BOX_TOP,     //pupil box position left in frame in unit of pixel
    DBG_META_PUPIL_BOX_WID,     //pupil box width  in unit of pixel
    DBG_META_PUPIL_BOX_HGT,     //pupil box height in unit of pixel
    DBG_META_TOTAL
};

/**
 * @brief  eye tracking debug image callback
 * @param   type: ignore
 *          img : gray scale eye camera image
 *          total : frame size in unit of bytes
 *          ts : time stamp
 *          meta : ignore
 *          refcon : user data
 */
typedef void (*PUB_DBG_CB)( int type, void *img, int total, int64_t ts, void *meta, void *refcon );

struct PUB;
/**
 * @brief  set eye tracking data callback
 * @param   pub: instance
 *          dbg_cb : debug image callback
 *          refcon : user data
 */
void pub_set_dbg_img_cb( struct PUB *pub, PUB_DBG_CB dbg_cb, void *refcon );

}
#endif  //__PUBDBGH__
