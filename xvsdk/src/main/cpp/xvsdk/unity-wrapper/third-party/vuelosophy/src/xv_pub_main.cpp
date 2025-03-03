//
//  Copyright (C) 2016-2022  Vuelosophy, Inc. - All Rights Reserved
//
// Eye Tracking Pubic API Test Harness
//
//
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pub.h"
#include "pub_dbg.h"
#include "xv_pub_main.h"
#include <android/log.h>

#define LOG_TAG "xv#vuelosophy"
#define LOG_DEBUG(...)                                                         \
  do {                                                                         \
    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__);              \
  } while (false)

#define ilog(...)                                                              \
  do {                                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
    fprintf(stderr, "\n");                                                     \
  } while (0)

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480
#define CAM_WIDTH 640
#define CAM_HEIGHT 480
#define CAM_FPS 60
#define ET_TOTAL 1
#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480
#define DFT_REPEAT 50
#define CONF_EVRY           30
#define CONF_FROM           100

static float g_conf_gaze_x[CALI_SAMPLE_TOTAL] = { 0.2,  0.8,  0.5 };
static float g_conf_gaze_y[CALI_SAMPLE_TOTAL] = { 0.2,  0.2,  0.8 };

typedef struct {
  int ft_wid;
  int ft_hgt;
  int et_wid;
  int et_hgt;
  int et_fps;
  int et_frame_size;
  int et_tot;
  uint8_t *et_img;
  int frame_wid;
  int frame_hgt;
  int frame_size;
  int frame_total;
  int s_idx;
  int s_cnt;

  FILE *f;
  uint8_t *data;
  int data_size;

  struct PUB *pub;
} AppState;

AppState g_app_state;
xv_pub_callback g_pub_callback;
xv_pub_dbg_callback g_pub_dbg_callback;
char g_path[256];

static void about() {
  char *str = "\n\n\
        vuelosopy eye tracking public api test harness\n\
        ./pub_sample yuv_path [et_total] [repeat]\n\
        et_total 1: one camera frame, 2: two camera frames, top-down layout\n\
        repeat: replay total times\n\
        contro-c to break\n\
        \n\
    ";
  ilog("%s", str);
}

static void pub_gaze_o_cb(float *gaze_o, int64_t ts, void *meta, void *refcon) {

    LOG_DEBUG( "into pub_gaze_o_cb, ts: %lld",  (long long)ts );
    if( gaze_o != NULL )  LOG_DEBUG( "pub_gaze_o_cb gaze_x:  %f, gaze_y:  %f",  gaze_o[0], gaze_o[1] );
    float *custom_meta = (float *)meta;
    int   et_idx    = (int)custom_meta[GAZE_META_ET_IDX];     //***UPDATE1
    int   et_num    = (int)custom_meta[GAZE_META_ET_NUM];
    int   has_pupil = (int)custom_meta[GAZE_META_HAS_PUPIL];
    float *pupl_o   = has_pupil ? (float *)&custom_meta[GAZE_META_PUPIL_X] : NULL;
    int   fix       = (int)custom_meta[GAZE_META_FIX];        //***UPDATE6
    LOG_DEBUG( "pub_gaze_o_cb et_idx: %d, et_num: %d, fix: %d", et_idx, et_num, fix );

    if( pupl_o != NULL )
        LOG_DEBUG( "pub_gaze_o_cb pupl_x: %f, pupl_y: %f", pupl_o[0], pupl_o[1] );

  float gaze_x = 0, gaze_y = 0;
  float pupl_x = 0, pupl_y = 0;

  if (gaze_o != NULL) {
    gaze_x = gaze_o[0];
    gaze_y = gaze_o[1];
  }

  if (pupl_o != NULL) {
    pupl_x = pupl_o[0];
    pupl_y = pupl_o[1];
  }

  if (g_pub_callback != NULL) {
    g_pub_callback(et_idx, gaze_x, gaze_y, pupl_x, pupl_y, ts);
  }
}

static void pub_dbg_img_cb(int type, void *data, int total, int64_t ts, void *meta_in,void *refcon) {
    LOG_DEBUG( "into pub_dbg_img_cb, ts: %lld",  (long long)ts );
    float *custom_meta = (float *)meta_in;  //***UPDATE2
    int   et_idx    = (int)custom_meta[DBG_META_ET_IDX];
    int   et_num    = (int)custom_meta[DBG_META_ET_NUM];
    int   has_pupil = (int)custom_meta[DBG_META_HAS_PUPIL];     //***UPDATE3
    float *pupl_o   = has_pupil ? (float *)&custom_meta[DBG_META_PUPIL_X] : NULL;
    int   box_lft   = (int)custom_meta[DBG_META_PUPIL_BOX_LFT];    //pupil box position top  in frame in unit of pixel
    int   box_top   = (int)custom_meta[DBG_META_PUPIL_BOX_TOP];     //pupil box position left in frame in unit of pixel
    int   box_wid   = (int)custom_meta[DBG_META_PUPIL_BOX_WID];     //pupil box width  in unit of pixel
    int   box_hgt   = (int)custom_meta[DBG_META_PUPIL_BOX_HGT];     //pupil box height in unit of pixel
    LOG_DEBUG( "pub_dbg_img_cb et_idx: %d, et_num: %d", et_idx, et_num );
    LOG_DEBUG( "pub_dbg_img_cb pupil box: lft: %d, top: %d, wid: %d, hgt: %d, ", box_lft, box_top, box_wid, box_hgt );
    if( pupl_o != NULL )
        LOG_DEBUG( "pub_dbg_img_cb pupl_x: %f, pupl_y: %f", pupl_o[0], pupl_o[1] );

   if (g_pub_dbg_callback != NULL) {
      g_pub_dbg_callback(et_idx, data, total);
   }
}

void xv_pub_init(int et_total, int screen_width, int screen_height,
                 int cam_width, int cam_height, int cam_fps) {
  LOG_DEBUG("xv_pub_init");
  g_app_state.ft_wid = screen_width;
  g_app_state.ft_hgt = screen_height;
  g_app_state.et_wid = cam_width;
  g_app_state.et_hgt = cam_height;
  g_app_state.et_fps = cam_fps;
  g_app_state.et_frame_size = g_app_state.et_wid * g_app_state.et_hgt;
  g_app_state.et_tot = et_total;
  g_app_state.frame_wid = g_app_state.et_wid;
  g_app_state.frame_hgt = g_app_state.et_hgt;
  g_app_state.frame_size = g_app_state.frame_wid * g_app_state.frame_hgt;

  int succeed = init_vue( g_path );
  LOG_DEBUG( "xv_pub_init path:%s, succeed:%d", g_path, succeed);

  g_app_state.pub = new_pub(g_app_state.ft_wid, g_app_state.ft_hgt, UNK_FOCLEN,
                            g_app_state.et_wid, g_app_state.et_hgt, UNK_FOCLEN,
                            g_app_state.et_fps, g_app_state.et_tot);
  if (g_app_state.pub == NULL) {
    LOG_DEBUG("xv_pub_init new_pub FAIL");
    dispose_pub(g_app_state.pub);
    return;
  }
  pub_set_gaze_o_cb(g_app_state.pub, pub_gaze_o_cb, (void *)&g_app_state);
  pub_set_dbg_img_cb(g_app_state.pub, pub_dbg_img_cb, (void *) &g_app_state);
}

void xv_pub_register(xv_pub_callback callback) { g_pub_callback = callback; }

void xv_pub_register_dbg(xv_pub_dbg_callback callback) { g_pub_dbg_callback = callback; }

void xv_pub_set_img(uint8_t *left_img, uint8_t *right_img, int size,
                    int64_t ts) {
  if (g_app_state.pub == NULL) {
    LOG_DEBUG("xv_pub_set_img FAIL: pub is null");
    return;
  }

  static int num = 0;
  pub_push_et_img(g_app_state.pub, 0, left_img, size, ts, num);
  pub_push_et_img(g_app_state.pub, 1, right_img, size, ts, num);
  num++;
  LOG_DEBUG("xv_pub_set_img %d", num);
}

int xv_pub_set_calibration(int eye, int index, float gaze_x, float gaze_y) {
  float conf_gaze_o[2] = { gaze_x, gaze_y };
  int has_oe_v[2]={0};
  float oe_v[2][3];

  LOG_DEBUG("xv_pub_set_calibration eye:%d index:%d =>(%f, %f)", eye, index, gaze_x, gaze_y);
  int ret = pub_set_conf_gaze_o(g_app_state.pub, eye, index, conf_gaze_o);
  has_oe_v[0] = pub_get_oe_v( g_app_state.pub, 0, oe_v[0] ); //***UPDATE5
  if( has_oe_v[0] ) LOG_DEBUG( "xv_pub_set_calibration oe_v[0], x: %f, y: %f, z: %f", oe_v[0][0],oe_v[0][1],oe_v[0][2] );
  return ret;
}

void xv_pub_dispose() {
  if (g_app_state.pub) {
    dispose_pub(g_app_state.pub);
    g_app_state.pub = NULL;
  }
}

void xv_pub_set_asset_path(const char* path) {
  LOG_DEBUG("xv_pub_set_asset_path %s", path);
  strcpy(g_path, path);
}

int pub_sample_main(char *yuv_path, int et_total, int play_repeat) {
  about();
  AppState *app = NULL;
  if (yuv_path == NULL) {
    ilog("yuv_path not specified!!!");
    return 0;
  }
  ilog("into pub_sample_main, yuv_path: %s, play_repeat: %d", yuv_path,
       play_repeat);
  app = (AppState *)malloc(sizeof(AppState));
  if (app == NULL)
    return 0;
  memset((void *)app, 0, sizeof(AppState));
  app->ft_wid = SCREEN_WIDTH;
  app->ft_hgt = SCREEN_HEIGHT;
  app->et_wid = CAM_WIDTH;
  app->et_hgt = CAM_HEIGHT;
  app->et_fps = CAM_FPS;
  app->et_frame_size = app->et_wid * app->et_hgt;
  app->et_tot = et_total;
  app->et_img = (uint8_t *)malloc(app->et_frame_size);
  app->frame_wid = app->et_wid;
  app->frame_hgt = app->et_hgt;
  app->frame_size = app->frame_wid * app->frame_hgt;
  app->s_idx = 0;
  app->s_cnt = 0;

  app->f = fopen(yuv_path, "rb");
  if (app->f == NULL) {
    ilog("yuv file: %s failed to open!!!", yuv_path);
    return 0;
  }
  fseek(app->f, 0, SEEK_END);
  app->data_size = (int)ftell(app->f);
  fseek(app->f, 0, SEEK_SET);
  app->data = (uint8_t *)malloc(app->data_size);
  if (app->data == NULL) {
    ilog("not enough memory!!!");
    return 0;
  }
  int len = (int)fread((void *)app->data, 1, app->data_size, app->f);
  if (len <= 0) {
    ilog("yuv file: %s failed to read!!!", yuv_path);
    return 0;
  }
  fclose(app->f);
  app->f = NULL;
  app->frame_total = app->data_size / app->frame_size;
  app->data_size = app->frame_total * app->frame_size;

  int succeed = init_vue( NULL );
  if( !succeed ) ilog( "init_vue failed!!!" );

  app->pub = new_pub(app->ft_wid, app->ft_hgt, UNK_FOCLEN, app->et_wid,
                     app->et_hgt, UNK_FOCLEN, app->et_fps, app->et_tot);
  if (app->pub == NULL) {
    ilog("public api init failure!!!");
    return 0;
  }
  pub_set_gaze_o_cb(app->pub, pub_gaze_o_cb, (void *)app);
  pub_set_dbg_img_cb( app->pub, pub_dbg_img_cb, (void *)app );
  //pub_set_sr_val( app->pub, 1 );  //***UPDATE7

  uint8_t *img = app->data;
  int64_t ts = 0;
  int dur = 1000000 / CAM_FPS;
  int cnt = 0;
  int num = 0;
  while (1) {
    uint8_t *this_img = img;
    int this_siz = app->frame_size;
    pub_push_et_img(app->pub, 0, this_img, this_siz, ts, num);
    img += app->frame_size;
    if (et_total == 2) {
      pub_push_et_img(app->pub, 1, this_img, this_siz, ts, num);
      img += app->frame_size;
    }
    if (num > CONF_FROM)   //calibration random test code
    {
      app->s_cnt++;
      if (app->s_cnt >= CONF_EVRY)   //calibration sample interval
      {
        app->s_cnt = 0; //loop back
        float conf_gaze_x = g_conf_gaze_x[app->s_idx]*app->ft_wid;
        float conf_gaze_y = g_conf_gaze_y[app->s_idx]*app->ft_hgt;
        float conf_gaze_o[2] = { conf_gaze_x, conf_gaze_y };
        int has_oe_v[2]={0};
        float oe_v[2][3];
        ilog( "#0 camera setting calibration point: %d", app->s_idx );
        pub_set_conf_gaze_o( app->pub, 0, app->s_idx, conf_gaze_o );
        has_oe_v[0] = pub_get_oe_v( app->pub, 0, oe_v[0] ); //***UPDATE5
        if( has_oe_v[0] ) ilog( "oe_v[0], x: %f, y: %f, z: %f", oe_v[0][0],oe_v[0][1],oe_v[0][2] );
        if (et_total == 2)
        {
          ilog( "#1 camera setting calibration point: %d", app->s_idx );
          pub_set_conf_gaze_o( app->pub, 1, app->s_idx, conf_gaze_o );
          has_oe_v[1] = pub_get_oe_v( app->pub, 0, oe_v[1] ); //if( !has_oe_v[0] )
          if( has_oe_v[1] ) ilog( "oe_v[1], x: %f, y: %f, z: %f", oe_v[1][0],oe_v[1][1],oe_v[1][2] );
        }
        app->s_idx++;
        if( app->s_idx >= CALI_SAMPLE_TOTAL ) app->s_idx = 0; //loop back
      }
    }
    usleep(dur);
    ts += dur;
    num++;
    if ((img - app->data) >= app->data_size) {
      ilog("\n...yuv back to beginning and keep going...            contro-c "
           "to break \n");
      img = app->data;
      cnt++;
      if (cnt >= play_repeat)
        break;
    }
  }
bail:
  ilog("pub_sample app closing down");
  if (app != NULL) {
    if (app->f != NULL)
      fclose(app->f);
    if (app->data != NULL)
      free(app->data);
    if (app->pub != NULL)
      dispose_pub(app->pub);
    free(app);
  }
  ilog("out of main");
  return 0;
}

#ifdef STANDALONE_BUILD
int main(int argc, const char **argv) {
  char *yuv_path = "../test/pub_sample/pub_sample.yuv";
  int et_total = ET_TOTAL;
  int play_repeat = DFT_REPEAT;
  if (argc >= 2) {
    yuv_path = (char *)argv[1];
    if (argc >= 3) {
      et_total = atoi(argv[2]);
      if (argc >= 4)
        play_repeat = atoi(argv[3]);
    }
  } else
    ilog("running in default settings...");
  return pub_sample_main(yuv_path, et_total, play_repeat);
}
#endif
