package com.k2fsa.sherpa.onnx;

import android.app.Activity;
import android.util.Log;

import com.unity3d.player.UnityPlayer;

public class UnityBleBridge {
    private static final String TAG = "UnityBleBridge";
    private static AsrUtils asrUtils;
    public  static void sendUnityMessage(String objectName,String mothodName,String args){
        try {
            UnityPlayer.UnitySendMessage(objectName, mothodName, args);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage());
        }
    }
    private  static  final String UNITY_OBJ_NAME = "Aitalk";
    public static void doResult(String result){
        sendUnityMessage(UNITY_OBJ_NAME,"onResult",result);
    }

    public static AsrUtils getInstance(Activity unityActivity) {
        if (asrUtils == null) {
            asrUtils = AsrUtils.getInstance(unityActivity);
        }
        return asrUtils;
    }
    public boolean initMicrophone(){
       return asrUtils.initMicrophone();
    }
    public void startAsrRecord(){
        asrUtils.startAsrRecord();
    }
    public void stopAsrRecord(){
        asrUtils.stopAsrRecord();
    }
}
