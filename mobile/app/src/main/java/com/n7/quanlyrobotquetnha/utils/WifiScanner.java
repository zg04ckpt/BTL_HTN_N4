package com.n7.quanlyrobotquetnha.utils;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.net.ConnectivityManager;
import android.net.NetworkCapabilities;
import android.net.DhcpInfo;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Build;

import java.util.List;
import java.util.Locale;
import java.util.function.BiConsumer;

public class WifiScanner {

    public static String getCurSSID(Context context) {
        WifiManager wifiManager = (WifiManager) context.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        String ssid = wifiManager.getConnectionInfo().getSSID();

        if (ssid == null) return "";

        if (ssid.startsWith("\"") && ssid.endsWith("\"") && ssid.length() >= 2) {
            ssid = ssid.substring(1, ssid.length() - 1);
        }

        if (ssid.equals("<unknown ssid>")) return "";

        return ssid;
    }

    public static boolean isWifiEnabled(Context context) {
        WifiManager wifiManager = (WifiManager) context.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        return wifiManager != null && wifiManager.isWifiEnabled();
    }

    public static boolean isNetworkAvailable(Context context) {
        ConnectivityManager cm = (ConnectivityManager) context.getSystemService(Context.CONNECTIVITY_SERVICE);
        if (cm == null) return false;

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            android.net.Network network = cm.getActiveNetwork();
            if (network == null) return false;
            NetworkCapabilities capabilities = cm.getNetworkCapabilities(network);
            return capabilities != null && (
                    capabilities.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) ||
                    capabilities.hasTransport(NetworkCapabilities.TRANSPORT_CELLULAR) ||
                    capabilities.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET));
        } else {
            android.net.NetworkInfo activeNetworkInfo = cm.getActiveNetworkInfo();
            return activeNetworkInfo != null && activeNetworkInfo.isConnected();
        }
    }

    public static String getWifiGatewayHost(Context context) {
        WifiManager wifiManager = (WifiManager) context.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        if (wifiManager == null || !wifiManager.isWifiEnabled()) {
            return "";
        }

        DhcpInfo dhcpInfo = wifiManager.getDhcpInfo();
        if (dhcpInfo == null || dhcpInfo.gateway == 0) {
            return "";
        }

        return intToIp(dhcpInfo.gateway);
    }

    public static void checkNotConfigStatus(Context context, String robotPrefix, BiConsumer<Boolean, String> callback) {
        WifiManager wifiManager = (WifiManager) context.getApplicationContext().getSystemService(Context.WIFI_SERVICE);

        if (wifiManager == null || !wifiManager.isWifiEnabled()) {
            callback.accept(false, "");
            return;
        }

        String currentSsid = getCurSSID(context);
        if (isRobotApSsid(currentSsid, robotPrefix)) {
            callback.accept(true, currentSsid);
            return;
        }

        BroadcastReceiver wifiScanReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                safeUnregister(context, this);
                List<ScanResult> results = wifiManager.getScanResults();
                for (ScanResult result : results) {
                    if (isRobotApSsid(result.SSID, robotPrefix)) {
                        callback.accept(true, result.SSID);
                        return;
                    }
                }
                callback.accept(false, "");
            }
        };

        context.registerReceiver(wifiScanReceiver, new IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION));
        boolean success = wifiManager.startScan();
        if (!success) {
            // Nếu startScan thất bại (do throttling), thử lấy kết quả cũ luôn
            List<ScanResult> results = wifiManager.getScanResults();
            for (ScanResult result : results) {
                if (isRobotApSsid(result.SSID, robotPrefix)) {
                    safeUnregister(context, wifiScanReceiver);
                    callback.accept(true, result.SSID);
                    return;
                }
            }

            safeUnregister(context, wifiScanReceiver);
            callback.accept(false, "");
        }
    }

    private static void safeUnregister(Context context, BroadcastReceiver receiver) {
        try {
            context.unregisterReceiver(receiver);
        } catch (IllegalArgumentException ignored) {
            // Receiver có thể đã được hủy trước đó.
        }
    }

    private static boolean isRobotApSsid(String ssid, String robotPrefix) {
        if (ssid == null || robotPrefix == null) {
            return false;
        }

        String normalizedSsid = ssid.trim().toLowerCase(Locale.ROOT);
        String normalizedPrefix = robotPrefix.trim().toLowerCase(Locale.ROOT);

        if (normalizedSsid.isEmpty() || normalizedPrefix.isEmpty()) {
            return false;
        }

        return normalizedSsid.startsWith(normalizedPrefix);
    }

    private static String intToIp(int value) {
        return (value & 0xFF) + "." +
                ((value >> 8) & 0xFF) + "." +
                ((value >> 16) & 0xFF) + "." +
                ((value >> 24) & 0xFF);
    }
}
