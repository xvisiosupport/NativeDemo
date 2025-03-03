package com.k2fsa.sherpa.onnx;

import android.content.res.AssetManager;

public class VadAsrWrapper {

    private long ptr;

    public VadAsrWrapper(AssetManager assetManager, String vadModelPath, String asrModelPath, String tokenPath) {
        ptr = newFromAsset(assetManager, vadModelPath, asrModelPath, tokenPath);
    }

    @Override
    protected void finalize() throws Throwable {
        try {
            if (ptr!= 0L) {
                delete(ptr);
                ptr = 0;
            }
        } finally {
            super.finalize();
        }
    }

    public void release() {
        try {
            finalize();
        } catch (Throwable e) {
            // 可以在这里添加对异常的处理逻辑，比如打印日志等
            e.printStackTrace();
        }
    }

    public String processAudio(float[] samples) {
        return processAudio(ptr, samples);
    }

    private native long newFromAsset(AssetManager assetManager, String vad_model, String asr_model, String tokens);

    private native void delete(long ptr);

    private native String processAudio(long ptr, float[] samples);

    static {
        System.loadLibrary("sherpa-onnx-jni");
    }
}