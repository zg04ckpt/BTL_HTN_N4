package com.n7.quanlyrobotquetnha.utils;

import java.util.concurrent.TimeUnit;

import okhttp3.Call;
import okhttp3.HttpUrl;
import okhttp3.Callback;
import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;

public class HttpClient {
    private static final String DEFAULT_LOCAL_BASE_URL = "http://192.168.1.1";
    // Mẫu Cloud URL - trong thực tế sẽ là server của bạn
    private static final String CLOUD_BASE_URL = "https://api.robot-cleaner-cloud.com"; 
    private static volatile String robotBaseUrl = DEFAULT_LOCAL_BASE_URL;
    
    private static OkHttpClient client;
    public static final MediaType JSON = MediaType.get("application/json; charset=utf-8");

    public static OkHttpClient getInstance() {
        if (client == null) {
            client = new OkHttpClient.Builder()
                    .connectTimeout(10, TimeUnit.SECONDS)
                    .readTimeout(10, TimeUnit.SECONDS)
                    .build();
        }
        return client;
    }

    public static String getRobotBaseUrl() {
        return robotBaseUrl;
    }

    public static boolean setRobotHost(String hostOrUrl) {
        String normalized = normalizeBaseUrl(hostOrUrl);
        if (normalized == null) {
            return false;
        }

        robotBaseUrl = normalized;
        return true;
    }

    public static Call getWithBaseUrl(String baseUrlOrHost, String endpoint, Callback callback) {
        String normalized = normalizeBaseUrl(baseUrlOrHost);
        if (normalized == null) {
            throw new IllegalArgumentException("Địa chỉ robot không hợp lệ");
        }

        Request request = new Request.Builder()
                .url(buildUrl(normalized, endpoint))
                .build();
        Call call = getInstance().newCall(request);
        call.enqueue(callback);
        return call;
    }

    public static void get(String endpoint, Callback callback) {
        Request request = new Request.Builder()
                .url(buildUrl(robotBaseUrl, endpoint))
                .build();
        getInstance().newCall(request).enqueue(callback);
    }

    public static void getFromCloud(String endpoint, Callback callback) {
        Request request = new Request.Builder()
                .url(buildUrl(CLOUD_BASE_URL, endpoint))
                .build();
        getInstance().newCall(request).enqueue(callback);
    }

    public static void post(String endpoint, String json, Callback callback) {
        RequestBody body = RequestBody.create(json, JSON);
        Request request = new Request.Builder()
                .url(buildUrl(robotBaseUrl, endpoint))
                .post(body)
                .build();
        getInstance().newCall(request).enqueue(callback);
    }

    private static String buildUrl(String baseUrl, String endpoint) {
        String safeEndpoint = endpoint == null ? "" : endpoint.trim();
        if (safeEndpoint.isEmpty()) {
            return baseUrl;
        }

        if (!safeEndpoint.startsWith("/")) {
            safeEndpoint = "/" + safeEndpoint;
        }

        return baseUrl + safeEndpoint;
    }

    private static String normalizeBaseUrl(String hostOrUrl) {
        if (hostOrUrl == null) {
            return null;
        }

        String raw = hostOrUrl.trim();
        if (raw.isEmpty()) {
            return null;
        }

        if (!raw.startsWith("http://") && !raw.startsWith("https://")) {
            raw = "http://" + raw;
        }

        HttpUrl parsed = HttpUrl.parse(raw);
        if (parsed == null || parsed.host() == null || parsed.host().isEmpty()) {
            return null;
        }

        String normalized = parsed.newBuilder()
                .encodedPath("/")
                .query(null)
                .fragment(null)
                .build()
                .toString();

        if (normalized.endsWith("/")) {
            normalized = normalized.substring(0, normalized.length() - 1);
        }

        return normalized;
    }
}
