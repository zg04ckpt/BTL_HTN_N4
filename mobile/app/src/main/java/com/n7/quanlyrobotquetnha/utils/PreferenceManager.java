package com.n7.quanlyrobotquetnha.utils;

import android.content.Context;
import android.content.SharedPreferences;

public class PreferenceManager {
    private static final String PREF_NAME = "QuanLyRobotLauNha";
    private static PreferenceManager instance;
    private final SharedPreferences sharedPreferences;
    private final SharedPreferences.Editor editor;

    public static final String KEY_ROBOT_IP = "robot_ip";
    public static final String KEY_LAST_CONFIG_WIFI_SSID = "last_config_wifi_ssid";

    private PreferenceManager(Context context) {
        sharedPreferences = context.getApplicationContext().getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
        editor = sharedPreferences.edit();
    }

    public static synchronized PreferenceManager getInstance(Context context) {
        if (instance == null) {
            instance = new PreferenceManager(context);
        }
        return instance;
    }

    public void putString(String key, String value) {
        editor.putString(key, value).apply();
    }

    public void putInt(String key, int value) {
        editor.putInt(key, value).apply();
    }

    public void putBoolean(String key, boolean value) {
        editor.putBoolean(key, value).apply();
    }

    // --- CÁC HÀM LẤY DỮ LIỆU ---

    public String getString(String key, String defaultValue) {
        return sharedPreferences.getString(key, defaultValue);
    }

    public int getInt(String key, int defaultValue) {
        return sharedPreferences.getInt(key, defaultValue);
    }

    public boolean getBoolean(String key, boolean defaultValue) {
        return sharedPreferences.getBoolean(key, defaultValue);
    }

    // --- XÓA DỮ LIỆU ---

    public void remove(String key) {
        editor.remove(key).apply();
    }

    public void clear() {
        editor.clear().apply();
    }
}