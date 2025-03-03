package com.k2fsa.sherpa.onnx;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.content.IntentFilter;
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;

import com.unity3d.player.UnityPlayer;
import java.util.concurrent.atomic.AtomicInteger;

public class AsrUtils {
    private static final String TAG = "LJASR";
    private AudioRecord audioRecord;
    private Thread recordingThread;

    private int sampleRateInHz = 16000;
    private int channelConfig = AudioFormat.CHANNEL_IN_MONO;
    // Note: We don't use AudioFormat.ENCODING_PCM_FLOAT
    // since the AudioRecord.read(float[]) needs API level >= 23
    // but we are targeting API level >= 21
    private int audioFormat = AudioFormat.ENCODING_PCM_16BIT;

    private int audioSource = MediaRecorder.AudioSource.MIC;
    private volatile boolean isRecording = false;
    private AtomicInteger idx = new AtomicInteger(0);
    private static AsrUtils mInstance;
    private String lastText = "";
    private Context mContext;
    private VadAsrWrapper vadAsrWrapper;

    public static synchronized AsrUtils getInstance(Activity activity) {
        if (mInstance == null) {
            mInstance = new AsrUtils(activity);
        }
        return mInstance;
    }

    private AsrUtils(Activity context) {
        mContext = context;
    }

    @SuppressLint("MissingPermission")
    public boolean initMicrophone() {
        vadAsrWrapper = new VadAsrWrapper(
                mContext.getApplicationContext().getAssets(),
                "model/silero_vad.onnx",
                "model/model.int8.onnx",
                "model/tokens.txt"
        );

        int numBytes = AudioRecord.getMinBufferSize(sampleRateInHz, channelConfig, audioFormat);
        Log.i(
                TAG, "buffer size in milliseconds: " + (numBytes * 1000.0f / sampleRateInHz)
        );

        audioRecord = new AudioRecord(
                audioSource,
                sampleRateInHz,
                channelConfig,
                audioFormat,
                numBytes * 2 // a sample has two bytes as we are using 16-bit PCM
        );
        return true;
    }

    public void resetIdx() {
        idx.set(0);
    }

    public void processSamples(VadAsrWrapper vadAsrWrapper) {
        Log.i(TAG, "processing samples");

        int bufferSize = 512; // in samples
        short[] buffer = new short[bufferSize];

        while (isRecording) {
            int ret = audioRecord.read(buffer, 0, buffer.length);
            if (ret > 0) {
                float[] samples = new float[ret];
                for (int i = 0; i < ret; i++) {
                    samples[i] = buffer[i] / 32768.0f;
                }
                String text = vadAsrWrapper.processAudio(samples);
                if (text != null && !text.isEmpty()) {
                    lastText = lastText + "\n" + idx.get() + ": " + text;
                    idx.incrementAndGet();
                }
                ((Activity) mContext).runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        //     tvResult.setText(lastText.toLowerCase());
                        UnityBleBridge.doResult(lastText.toLowerCase());
                        Log.i(TAG, "processing samples lastText is " + lastText.toLowerCase());
                    }
                });
            }
        }
    }

    public void startAsrRecord() {
        if (!isRecording) {

            Log.i(TAG, "state: " + audioRecord.getState());
            audioRecord.startRecording();
            //   startAsr.setText("stopAsr");
            isRecording = true;


            //    lastText = "";
            resetIdx();

            //vad.reset();

            recordingThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    processSamples(vadAsrWrapper);
                }
            });
            recordingThread.start();
            Log.i(TAG, "Started recording");
        }
    }
    public void stopAsrRecord() {
            isRecording = false;
            audioRecord.stop();
            audioRecord.release();
            audioRecord = null;

            //     startAsr.setText("startAsr");
            Log.i(TAG, "Stopped recording");
    }

}

