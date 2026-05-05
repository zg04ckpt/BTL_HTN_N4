package com.n7.quanlyrobotquetnha.utils;

import android.os.Handler;
import android.os.Looper;

import androidx.annotation.NonNull;

import com.google.gson.JsonObject;

import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.concurrent.TimeUnit;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;

public class RobotSocketManager {

    public interface Listener {
        default void onSocketConnected() {
        }

        default void onSocketDisconnected(String reason) {
        }

        default void onSocketMessage(String text) {
        }
    }

    public interface ConnectCallback {
        void onConnected();

        void onFailure(String reason);
    }

    private static RobotSocketManager instance;

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final Set<Listener> listeners = new CopyOnWriteArraySet<>();
    private final OkHttpClient client;

    private volatile WebSocket webSocket;
    private volatile boolean connected = false;
    private volatile String socketUrl = "";

    private RobotSocketManager() {
        client = new OkHttpClient.Builder()
                .connectTimeout(8, TimeUnit.SECONDS)
                .readTimeout(0, TimeUnit.MILLISECONDS)
                .build();
    }

    public static synchronized RobotSocketManager getInstance() {
        if (instance == null) {
            instance = new RobotSocketManager();
        }
        return instance;
    }

    public synchronized void connect(String robotBaseUrl, ConnectCallback callback) {
        String nextSocketUrl = toSocketUrl(robotBaseUrl);
        if (nextSocketUrl == null || nextSocketUrl.isEmpty()) {
            if (callback != null) {
                mainHandler.post(() -> callback.onFailure("Địa chỉ socket robot không hợp lệ"));
            }
            return;
        }

        if (connected && nextSocketUrl.equals(socketUrl) && webSocket != null) {
            if (callback != null) {
                mainHandler.post(callback::onConnected);
            }
            return;
        }

        disconnectInternal("reconnect", false);
        socketUrl = nextSocketUrl;

        Request request = new Request.Builder().url(socketUrl).build();
        webSocket = client.newWebSocket(request, new WebSocketListener() {
            private boolean callbackResolved = false;

            @Override
            public void onOpen(@NonNull WebSocket webSocket, @NonNull Response response) {
                connected = true;
                notifyConnected();
                if (!callbackResolved && callback != null) {
                    callbackResolved = true;
                    mainHandler.post(callback::onConnected);
                }
            }

            @Override
            public void onMessage(@NonNull WebSocket webSocket, @NonNull String text) {
                notifyMessage(text);
            }

            @Override
            public void onClosed(@NonNull WebSocket webSocket, int code, @NonNull String reason) {
                connected = false;
                notifyDisconnected(reason);
            }

            @Override
            public void onFailure(@NonNull WebSocket webSocket, @NonNull Throwable t, Response response) {
                connected = false;
                String reason = t.getMessage() == null ? "Socket lỗi không rõ" : t.getMessage();
                notifyDisconnected(reason);
                if (!callbackResolved && callback != null) {
                    callbackResolved = true;
                    mainHandler.post(() -> callback.onFailure(reason));
                }
            }
        });
    }

    public synchronized void disconnect(String reason) {
        disconnectInternal(reason, true);
    }

    public synchronized boolean sendText(String text) {
        if (!connected || webSocket == null) {
            return false;
        }

        return webSocket.send(text);
    }

    public boolean sendJson(JsonObject jsonObject) {
        return sendText(jsonObject.toString());
    }

    public boolean isConnected() {
        return connected;
    }

    public void addListener(Listener listener) {
        if (listener != null) {
            listeners.add(listener);
        }
    }

    public void removeListener(Listener listener) {
        if (listener != null) {
            listeners.remove(listener);
        }
    }

    private synchronized void disconnectInternal(String reason, boolean userInitiated) {
        WebSocket oldSocket = webSocket;
        webSocket = null;
        connected = false;

        if (oldSocket != null) {
            oldSocket.close(1000, reason == null ? "closed" : reason);
        }

        if (userInitiated) {
            notifyDisconnected(reason == null ? "Đã ngắt kết nối" : reason);
        }
    }

    private void notifyConnected() {
        mainHandler.post(() -> {
            for (Listener listener : listeners) {
                listener.onSocketConnected();
            }
        });
    }

    private void notifyDisconnected(String reason) {
        mainHandler.post(() -> {
            for (Listener listener : listeners) {
                listener.onSocketDisconnected(reason == null ? "Socket đã ngắt" : reason);
            }
        });
    }

    private void notifyMessage(String text) {
        mainHandler.post(() -> {
            for (Listener listener : listeners) {
                listener.onSocketMessage(text);
            }
        });
    }

    private String toSocketUrl(String robotBaseUrl) {
        if (robotBaseUrl == null) {
            return null;
        }

        String base = robotBaseUrl.trim();
        if (base.isEmpty()) {
            return null;
        }

        String socket;
        if (base.startsWith("https://")) {
            socket = "wss://" + base.substring("https://".length());
        } else if (base.startsWith("http://")) {
            socket = "ws://" + base.substring("http://".length());
        } else if (base.startsWith("wss://") || base.startsWith("ws://")) {
            socket = base;
        } else {
            socket = "ws://" + base;
        }

        if (socket.endsWith("/")) {
            socket = socket.substring(0, socket.length() - 1);
        }

        return socket + "/ws";
    }
}