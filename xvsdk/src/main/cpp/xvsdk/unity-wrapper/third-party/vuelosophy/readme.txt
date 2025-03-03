- built for android-24

- to extract:
$ tar xvf an.tgz

- api and sample code:
$ ls an/api/
  pub.h,
  pub_dbg.h,
  pub_sample_main.c

- to copy binaries to android device
$ cd an/bin/armeabi-v7a
or
$ cd an/bin/arm64-v8a
$ adb push * /data/local/tmp

- to copy test file:
$ adb push pub_sample.yuv /data/local/tmp

- to run
$ adb shell
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./
$ cd /data/local/tmp
$ ./pub_sample ./pub_sample.yuv 1
or
$ ./pub_sample ./pub_sample.yuv 2


- releasenote:
2022_11_16 PDT
1. turn on slippage resistance
2. changes in gaze and dgb custom meta format,
   see ***UPDATE1, ***UPDATE2 in pub_sample_main.c for details

2022_11_28 PDT
1. improve API, output pupil position, pupil box and eye center position
   see ***UPDATE3, ***UPDATE4, ***UPDATE5  in pub_sample_main.c for details

2022_11_30 PDT
1. output fixation
   see ***UPDATE6  in pub_sample_main.c for details
2. example of setting slippage resistance value
   see ***UPDATE7  in pub_sample_main.c for details

2022_12_1 PDT
1. add data dir in pub api

2022_12_2 PDT
1. move data dir in separate pub api
