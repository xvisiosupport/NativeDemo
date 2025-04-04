package org.xvisio.xslam;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.RadioGroup;
import android.widget.TextView;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.constraintlayout.widget.ConstraintLayout;

//import com.hjq.permissions.OnPermissionCallback;
//import com.hjq.permissions.Permission;
//import com.hjq.permissions.XXPermissions;

import com.iflytek.cloud.SpeechError;

import com.iflytek.cloud.WakeuperListener;
import com.iflytek.cloud.WakeuperResult;

import com.k2fsa.sherpa.onnx.AsrUtils;
import com.xv.aitalk.AiTalkListener;
import com.xv.aitalk.AsrInstance;

import org.json.JSONException;
import org.json.JSONObject;
import org.xvisio.xvsdk.DeviceListener;
import org.xvisio.xvsdk.ImuListener;
import org.xvisio.xvsdk.PoseListener;
import org.xvisio.xvsdk.RgbListener;
import org.xvisio.xvsdk.SgbmListener;
import org.xvisio.xvsdk.StereoListener;
import org.xvisio.xvsdk.StreamData;
import org.xvisio.xvsdk.TofIrListener;
import org.xvisio.xvsdk.TofListener;
import org.xvisio.xvsdk.XCamera;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "XVSDK Demo";

    private static final int PERMISSIONS_REQUEST_CAMERA = 0;
    private boolean mPermissionsGranted = false;
    private static final  String  LOCAL_BNF = "#BNF+IAT 1.0 UTF-8;\n"
            +"!grammar word;\n"
            +"!slot <words>;\n"
            +"!start <words>;\n"
            +"<words>:篮球!id(1000)|足球!id(1000)|救护车!id(1001)|放大!id(1002)|缩小!id(1003)|旋转!id(1004);\n";
    private Context mAppContext = null;
    private XCamera mCamera = null;
    private String[] permissions = {Manifest.permission.RECORD_AUDIO};
    private boolean isMixedMode = true;
    private boolean isCustomerAsr = false;
    private static final int REQUEST_RECORD_AUDIO_PERMISSION = 200;
    int rgbSolution = 1;
    int cameraSelect = 0;
    int modeSelect = 0;
    private AsrInstance asrInstance = null;
    Button mBtRgbSolution;
    Button mBtStartASR;
    ImageView mIvStream;
    ConstraintLayout mClTof;
    TextView mTvSolution, mTvFps,mTvResult;
    private StreamHandler mMainHandler;
    private Activity mContext;
    private volatile boolean isRecording = false;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mAppContext = getApplicationContext();
        mContext = this;
        mMainHandler = new StreamHandler(Looper.getMainLooper());
        setContentView(R.layout.activity_main);

        mBtRgbSolution = findViewById(R.id.button_rgb);
        mBtRgbSolution.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showRgbSolutionDialog();
            }
        });
        mBtStartASR = findViewById(R.id.bt_startAsr);

        RadioGroup radioSlam = findViewById(R.id.radio_slam_mode);
        modeSelect = radioSlam.getCheckedRadioButtonId();
        radioSlam.setOnCheckedChangeListener(mSlamModeListener);

        CheckBox checkBoxSlam = findViewById(R.id.checkbox_slam);
        checkBoxSlam.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if (b) {
                    mCamera.startStream(XCamera.Stream.SLAM);
                } else {
                    mCamera.stopStream(XCamera.Stream.SLAM);
                }
            }
        });

        CheckBox checkBoxImu = findViewById(R.id.checkbox_imu);
        checkBoxImu.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if (b) {
                    mCamera.startStream(XCamera.Stream.IMU);
                } else {
                    mCamera.stopStream(XCamera.Stream.IMU);
                }
            }
        });
        //初始化自定义语音识别model资源
        if(isCustomerAsr){
            boolean ret = AsrUtils.getInstance(this).initMicrophone();
            if (!ret) {
                Log.e(TAG, "Failed to initialize microphone");
                return;
            }
        }


//        AsrUtils.getInstance().processSamples(vadAsrWrapper);
        mIvStream = findViewById(R.id.rgbView);
        mClTof = findViewById(R.id.cl_tof);
        mTvSolution = findViewById(R.id.tv_rgb_solution);
        mTvResult = findViewById(R.id.tv_result);
        mTvFps = findViewById(R.id.tv_rgb_fps);
        RadioGroup cameraRadio = findViewById(R.id.radio_camera);
        cameraSelect = cameraRadio.getCheckedRadioButtonId();
        cameraRadio.setOnCheckedChangeListener(mCemeraSelectListener);

        String[] permissions = new String[]{Manifest.permission.READ_EXTERNAL_STORAGE,Manifest.permission.RECORD_AUDIO, Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.CAMERA};
        requestPermissions(permissions, 101);
        mBtStartASR.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                setupAitalk(mContext);
            }
        });

        //uvc permission https://github.com/saki4510t/UVCPermissionTest
//        XXPermissions.with(this)
//                // 不适配 Android 11 可以这样写
//                .permission(Permission.Group.STORAGE)
//                // 适配 Android 11 需要这样写，这里无需再写 Permission.Group.STORAGE
//                // .permission(Permission.MANAGE_EXTERNAL_STORAGE)
//                .permission(Permission.CAMERA)
//                .request(new OnPermissionCallback() {
//
//                    @Override
//                    public void onGranted(List<String> permissions, boolean all) {
//                        if (all) {
//                            mPermissionsGranted = true;
//                            init();
//                            Log.i("test", "startRecord 0=");
//
////							Toast.makeText(mActivityContext,"获取存储权限成功", (int)1000).show();
//                        }
//                    }
//
//                    @Override
//                    public void onDenied(List<String> permissions, boolean never) {
//                        if (never) {
//                            Toast.makeText(mAppContext, "被永久拒绝授权，请手动授予存储权限", 1000).show();
//                            // 如果是被永久拒绝就跳转到应用权限系统设置页面
//                            XXPermissions.startPermissionActivity(mAppContext, permissions);
//                        } else {
//                            Toast.makeText(mAppContext, "获取存储权限失败", 1000).show();
//                        }
//                    }
//                });
//
//        mPermissionsGranted = true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.CAMERA}, PERMISSIONS_REQUEST_CAMERA);
            return;
        }
        mPermissionsGranted = true;
        init();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mPermissionsGranted) {
            init();
        } else {
            Log.e(TAG, "missing permissions");
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mCamera != null) {
            mCamera.stopStreams();
        }
    }

    private void init() {
        if (mCamera == null) {
            mCamera = new XCamera();
            mCamera.init(mAppContext);
            mCamera.setDevicesChangedCallback(mListener);
            mCamera.setImuCallback(mImuListener);
            mCamera.setStereoCallback(mStereoListener);
            mCamera.setRgbCallback(mRgbListener);
            mCamera.setTofCallback(mTofListener);
            mCamera.setTofIrCallback(mTofIrListener);
            mCamera.setSgbmCallback(mSgbmListener);
            mCamera.setPoseCallback(mPoseListener);

        }
        mCamera.setRgbSolution(rgbSolution);
    }

    RadioGroup.OnCheckedChangeListener mSlamModeListener = new RadioGroup.OnCheckedChangeListener() {
        @Override
        public void onCheckedChanged(RadioGroup radioGroup, int i) {
            if (modeSelect == i) {
                return;
            }
            modeSelect = i;

            switch (i) {
                case R.id.radio_mixed:
                    isMixedMode = true;
                    break;

                case R.id.radio_edge:
                    isMixedMode = false;
                    break;

                default:
                    break;
            }

            mCamera.setSlamMode(isMixedMode ? 0 : 2);
        }
    };

    RadioGroup.OnCheckedChangeListener mCemeraSelectListener = new RadioGroup.OnCheckedChangeListener() {
        @Override
        public void onCheckedChanged(RadioGroup radioGroup, int i) {
            if (cameraSelect == i) {
                return;
            }
            cameraSelect = i;
            mCamera.stopStream(XCamera.Stream.RGB);
            mCamera.stopStream(XCamera.Stream.TOF);
            mCamera.stopStream(XCamera.Stream.STEREO);
            mCamera.stopStream(XCamera.Stream.SGBM);

            switch (i) {
                case R.id.radio_rgb:
                    mCamera.startStream(XCamera.Stream.RGB);
                    break;

                case R.id.radio_tof:
                    mCamera.startStream(XCamera.Stream.TOF);
                    break;

                case R.id.radio_stereo:
                    mCamera.startStream(XCamera.Stream.STEREO);
                    break;

                case R.id.radio_sgbm:
                    mCamera.startStream(XCamera.Stream.SGBM);
                    break;
                default:
                    break;
            }
        }
    };

    public void testFunc(View view) {
        mCamera.testFuncs();
    }

    class StreamHandler extends Handler {

        public static final int RGB = 1;
        public static final int TOF = 2;
        public static final int STEREO = 3;
        public static final int SGBM = 4;
        public static final int TOF_IR = 5;

        public StreamHandler(Looper looper) {
            super(looper);
        }

        @Override
        public void handleMessage(Message msg) {
            Log.e(TAG, "handleMessage " + msg.what);
            switch (msg.what) {
                case RGB:
                    mBtRgbSolution.setVisibility(View.VISIBLE);
                    mIvStream.setVisibility(View.VISIBLE);
                    mClTof.setVisibility(View.GONE);
                    onRgbCallback((StreamData) msg.obj);
                    break;

                case TOF:
                    mBtRgbSolution.setVisibility(View.GONE);
                    mIvStream.setVisibility(View.GONE);
                    mClTof.setVisibility(View.VISIBLE);
                    onTofCallback((StreamData) msg.obj);
                    break;

                case STEREO:
                    mBtRgbSolution.setVisibility(View.GONE);
                    mIvStream.setVisibility(View.VISIBLE);
                    mClTof.setVisibility(View.GONE);
                    onStereoCallback((StreamData) msg.obj);
                    break;

                case SGBM:
                    mBtRgbSolution.setVisibility(View.GONE);
                    mIvStream.setVisibility(View.VISIBLE);
                    mClTof.setVisibility(View.GONE);
                    onSgbmCallback((StreamData) msg.obj);
                    break;

                case TOF_IR:
                    mBtRgbSolution.setVisibility(View.GONE);
                    mIvStream.setVisibility(View.GONE);
                    mClTof.setVisibility(View.VISIBLE);
                    onTofIrCallback((StreamData) msg.obj);
                    break;

                default:
                    break;
            }
        }
    }

    public void showRgbSolutionDialog() {
        String[] rgbSolutionItems = {"1920x1080", "1280x720", "640x480"};
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setSingleChoiceItems(rgbSolutionItems, rgbSolution, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                rgbSolution = which;
                if (mCamera != null) {
                    mCamera.setRgbSolution(which);
                }
                dialog.dismiss();
            }
        });
        builder.show();
    }

    private final DeviceListener mListener = new DeviceListener() {
        @Override
        public void onDeviceAttach() {
        }

        @Override
        public void onDeviceDetach() {
        }
    };

    private final PoseListener mPoseListener = new PoseListener() {
        @Override
        public void onPose(final double x, final double y, final double z, final double pitch, final double yaw, final double roll) {
            mMainHandler.post(new Runnable() {
                @Override
                public void run() {
                    PoseDisplay pose_display = findViewById(R.id.poseDisplay);
                    pose_display.setValue(x, y, z, pitch, yaw, roll);
                }
            });

        }
    };

    private final ImuListener mImuListener = new ImuListener() {
        @Override
        public void onImu(final double x, final double y, final double z) {
            mMainHandler.post(new Runnable() {
                @Override
                public void run() {
                    AccelerometerDisplay a = findViewById(R.id.accelerometerDisplay);
                    a.setValues(x, y, z);
                }
            });
        }
    };

    private void onStereoCallback(StreamData data) {
        mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        mTvFps.setText("");
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());
        mIvStream.setImageBitmap(bitmap);
    }

    private final StereoListener mStereoListener = new StereoListener() {
        @Override
        public void onStereo(final int width, final int height, final int[] pixels) {
            sendStreamMessage(StreamHandler.STEREO, new StreamData(width, height, pixels));
        }
    };

    private void sendStreamMessage(int what, StreamData data) {
        mMainHandler.obtainMessage(what, data).sendToTarget();
    }

    private void onRgbCallback(StreamData data) {
        mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());

        if (data.getHeight() == 480) {
            mIvStream.setImageBitmap(bitmap);
            return;
        }

        float scale = data.getHeight() == 720 ? 0.75f : 0.5f;
        Bitmap scaled = Bitmap.createScaledBitmap(bitmap, (int) (data.getWidth() * scale), (int) (data.getHeight() * scale), false);
        bitmap.recycle();
        mIvStream.setImageBitmap(scaled);
    }

    private final RgbListener mRgbListener = new RgbListener() {
        @Override
        public void onFps(final int fps) {
            mMainHandler.post(new Runnable() {
                @Override
                public void run() {
                    mTvFps.setVisibility(View.VISIBLE);
                    mTvFps.setText("FPS:  " + fps);
                }
            });
        }

        @Override
        public void onRgb(final int width, final int height, final int[] pixels) {
            sendStreamMessage(StreamHandler.RGB, new StreamData(width, height, pixels));
        }
    };

    private void onSgbmCallback(StreamData data) {
        mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        mTvFps.setText("");
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());
        mIvStream.setImageBitmap(bitmap);
    }

    private final SgbmListener mSgbmListener = new SgbmListener() {
        @Override
        public void onSgbm(int width, int height, int[] pixels) {
            sendStreamMessage(StreamHandler.SGBM, new StreamData(width, height, pixels));
        }
    };

    private void onTofCallback(StreamData data) {
        mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        mTvFps.setText("");
        ImageView iv = mClTof.findViewById(R.id.tofView);
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());
        iv.setImageBitmap(bitmap);
    }

    private final TofListener mTofListener = new TofListener() {
        @Override
        public void onTof(int width, int height, int[] pixels) {
            sendStreamMessage(StreamHandler.TOF, new StreamData(width, height, pixels));
        }
    };

    private void onTofIrCallback(StreamData data) {
        // mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        mTvFps.setText("");
        ImageView iv = mClTof.findViewById(R.id.tofView2);
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());
        iv.setImageBitmap(bitmap);
    }

    private final TofIrListener mTofIrListener = new TofIrListener() {
        @Override
        public void onTofIr(int width, int height, int[] pixels) {
            sendStreamMessage(StreamHandler.TOF_IR, new StreamData(width, height, pixels));
        }
    };


    private void setupAitalk(Context context){
        asrInstance = new AsrInstance(context,false);
        asrInstance.init();
        mBtStartASR.setText("stopAsr");
        asrInstance.setWakeupListener(new WakeuperListener() {
            @Override
            public void onBeginOfSpeech() {

            }

            @Override
            public void onResult(WakeuperResult result) {
                Log.d(TAG,"WakeuperListener onResult....");
                try {
                    String text = result.getResultString();
                    //    UnityInterface.onAvwResult(text);
                    Log.d(TAG,"onResult...." + text);
                    JSONObject object;
                    object = new JSONObject(text);
                    StringBuffer buffer = new StringBuffer();
                    buffer.append("【RAW】 " + text);
                    buffer.append("\n");
                    buffer.append("【操作类型】" + object.optString("sst"));
                    buffer.append("\n");
                    buffer.append("【唤醒词id】" + object.optString("id"));
                    buffer.append("\n");
                    buffer.append("【得分】" + object.optString("score"));
                    buffer.append("\n");
                    buffer.append("【前端点】" + object.optString("bos"));
                    buffer.append("\n");
                    buffer.append("【尾端点】" + object.optString("eos"));
                 //   doResult( object.optString("keyword"),Integer.parseInt(object.optString("score")) , Integer.parseInt(object.optString("id")));
                    asrInstance.setWakeupState(true);
                    //customer asr 开关
                    if(isCustomerAsr){
                        AsrUtils.getInstance(mContext).startAsrRecord();
                    } else {
                        //科大讯飞命令词识别开关
                        asrInstance.startASR();
                    }
//                    mBtStartASR.setText("stopAsr");
                //
                } catch (JSONException e) {

                    e.printStackTrace();
                }
            }

            @Override
            public void onError(SpeechError speechError) {

            }

            @Override
            public void onEvent(int i, int i1, int i2, Bundle bundle) {

            }

            @Override
            public void onVolumeChanged(int i) {

            }
        });

        asrInstance.buildGrammar("local",LOCAL_BNF);
        asrInstance.setParam("local","word","60","5000","2000");
        asrInstance.setUseKeepAlive(false);
        //       asrInstance.startASR();
        asrInstance.startAvw(false);
    }
}
