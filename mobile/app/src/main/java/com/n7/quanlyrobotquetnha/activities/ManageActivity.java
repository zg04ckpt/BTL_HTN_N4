package com.n7.quanlyrobotquetnha.activities;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.Fragment;
import androidx.swiperefreshlayout.widget.SwipeRefreshLayout;

import com.google.android.material.bottomnavigation.BottomNavigationView;
import com.google.gson.JsonObject;
import com.n7.quanlyrobotquetnha.MainActivity;
import com.n7.quanlyrobotquetnha.R;
import com.n7.quanlyrobotquetnha.fragments.FavoriteFragment;
import com.n7.quanlyrobotquetnha.fragments.HomeFragment;
import com.n7.quanlyrobotquetnha.fragments.SettingsFragment;
import com.n7.quanlyrobotquetnha.utils.HttpClient;
import com.n7.quanlyrobotquetnha.utils.PreferenceManager;
import com.n7.quanlyrobotquetnha.utils.RobotUdpDiscovery;
import com.n7.quanlyrobotquetnha.utils.RobotSocketManager;

import okhttp3.HttpUrl;

public class ManageActivity extends AppCompatActivity implements RobotSocketManager.Listener {

    private TextView tvSocketState;
    private BottomNavigationView bottomNavigation;
    private SwipeRefreshLayout swipeRefresh;
    private final Handler refreshHandler = new Handler(Looper.getMainLooper());

    private boolean initialTabLoaded = false;
    private boolean discoveryInProgress = false;
    private RobotUdpDiscovery robotUdpDiscovery;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_manage);

        tvSocketState = findViewById(R.id.tvSocketState);
        bottomNavigation = findViewById(R.id.bottomNavigation);
        swipeRefresh = findViewById(R.id.swipeRefresh);
        swipeRefresh.setColorSchemeResources(R.color.purple_700);
        swipeRefresh.setProgressBackgroundColorSchemeResource(R.color.gray_100);

        bottomNavigation.setOnItemSelectedListener(item -> {
            int id = item.getItemId();
            return switchToTab(id);
        });

        swipeRefresh.setOnRefreshListener(() -> {
            int selectedTab = bottomNavigation.getSelectedItemId();
            requestRobotSnapshot();
            switchToTab(selectedTab);
            refreshHandler.postDelayed(() -> swipeRefresh.setRefreshing(false), 600);
        });

        ensureSocketConnected();
    }

    @Override
    protected void onStart() {
        super.onStart();
        RobotSocketManager.getInstance().addListener(this);
    }

    @Override
    protected void onStop() {
        RobotSocketManager.getInstance().removeListener(this);
        super.onStop();
    }

    @Override
    protected void onResume() {
        super.onResume();
        requestRobotSnapshot();
    }

    private void ensureSocketConnected() {
        if (RobotSocketManager.getInstance().isConnected()) {
            updateSocketState("Socket: đã kết nối");
            ensureDefaultTab();
            requestRobotSnapshot();
            return;
        }

        updateSocketState("Socket: đang kết nối...");
        String savedHost = PreferenceManager.getInstance(this)
                .getString(PreferenceManager.KEY_ROBOT_IP, "");
        if (savedHost.isEmpty()) {
            Toast.makeText(this, "Không có địa chỉ robot đã lưu", Toast.LENGTH_SHORT).show();
            finish();
            return;
        }

        if (!isLanHost(savedHost)) {
            updateSocketState("Socket: host cũ không thuộc LAN, đang quét lại...");
            discoverRobotAndConnect();
            return;
        }

        connectSocketWithHost(savedHost, true);
    }

    private void connectSocketWithHost(String host, boolean allowDiscoveryFallback) {
        if (!HttpClient.setRobotHost(host)) {
            if (allowDiscoveryFallback) {
                discoverRobotAndConnect();
            } else {
                Toast.makeText(this, "Địa chỉ robot không hợp lệ", Toast.LENGTH_SHORT).show();
                finish();
            }
            return;
        }

        if (!isLanHost(HttpClient.getRobotBaseUrl())) {
            if (allowDiscoveryFallback) {
                discoverRobotAndConnect();
            } else {
                Toast.makeText(this, "Socket chỉ hỗ trợ mạng nội bộ", Toast.LENGTH_SHORT).show();
                finish();
            }
            return;
        }

        RobotSocketManager.getInstance().connect(HttpClient.getRobotBaseUrl(), new RobotSocketManager.ConnectCallback() {
            @Override
            public void onConnected() {
                runOnUiThread(() -> {
                    updateSocketState("Socket: đã kết nối");
                    ensureDefaultTab();
                    requestRobotSnapshot();
                });
            }

            @Override
            public void onFailure(String reason) {
                runOnUiThread(() -> {
                    if (allowDiscoveryFallback) {
                        updateSocketState("Socket: host cũ lỗi, đang quét LAN...");
                        discoverRobotAndConnect();
                    } else {
                        updateSocketState("Socket: lỗi kết nối");
                        Toast.makeText(ManageActivity.this, "Không thể kết nối socket: " + reason, Toast.LENGTH_SHORT).show();
                        finish();
                    }
                });
            }
        });
    }

    private void discoverRobotAndConnect() {
        if (discoveryInProgress) {
            return;
        }
        discoveryInProgress = true;

        if (robotUdpDiscovery != null) {
            robotUdpDiscovery.stopDiscovery();
        }

        robotUdpDiscovery = new RobotUdpDiscovery(this, new RobotUdpDiscovery.DiscoveryCallback() {
            @Override
            public void onRobotFound(String ip) {
                runOnUiThread(() -> {
                    discoveryInProgress = false;
                    PreferenceManager.getInstance(ManageActivity.this)
                            .putString(PreferenceManager.KEY_ROBOT_IP, ip);
                    connectSocketWithHost(ip, false);
                });
            }

            @Override
            public void onError(String message) {
                runOnUiThread(() -> {
                    discoveryInProgress = false;
                    updateSocketState("Socket: không tìm thấy robot trong LAN");
                    Toast.makeText(ManageActivity.this,
                            "Không tìm thấy robot trong LAN để mở socket",
                            Toast.LENGTH_SHORT).show();
                    finish();
                });
            }
        });

        robotUdpDiscovery.startDiscovery();
    }

    private void ensureDefaultTab() {
        if (initialTabLoaded) {
            return;
        }
        initialTabLoaded = true;
        bottomNavigation.setSelectedItemId(R.id.menu_status);
    }

    private boolean switchToTab(int tabId) {
        if (tabId == R.id.menu_connect) {
            goToConnectPage();
            return true;
        }

        Fragment fragment = createFragmentForTab(tabId);
        if (fragment == null) {
            return false;
        }

        switchFragment(fragment);
        return true;
    }

    private Fragment createFragmentForTab(int tabId) {
        if (tabId == R.id.menu_status) {
            return new HomeFragment();
        }

        if (tabId == R.id.menu_config) {
            return new SettingsFragment();
        }

        if (tabId == R.id.menu_debug) {
            return new FavoriteFragment();
        }

        return null;
    }

    private void goToConnectPage() {
        RobotSocketManager.getInstance().disconnect("back_to_connect");
        Intent intent = new Intent(this, MainActivity.class);
        intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK);
        startActivity(intent);
        finish();
    }

    private void switchFragment(Fragment fragment) {
        getSupportFragmentManager()
                .beginTransaction()
                .replace(R.id.fragmentContainer, fragment)
                .commit();
    }

    private void requestRobotSnapshot() {
        JsonObject getStatus = new JsonObject();
        getStatus.addProperty("type", "get_status");
        RobotSocketManager.getInstance().sendJson(getStatus);

        JsonObject getConfig = new JsonObject();
        getConfig.addProperty("type", "get_config");
        RobotSocketManager.getInstance().sendJson(getConfig);
    }

    private void updateSocketState(String text) {
        tvSocketState.setText(text);
    }

    @Override
    public void onSocketConnected() {
        updateSocketState("Socket: đã kết nối");
    }

    @Override
    public void onSocketDisconnected(String reason) {
        updateSocketState("Socket: mất kết nối");
    }

    @Override
    protected void onDestroy() {
        if (robotUdpDiscovery != null) {
            robotUdpDiscovery.stopDiscovery();
        }
        super.onDestroy();
    }

    private boolean isLanHost(String hostOrUrl) {
        String host = extractHost(hostOrUrl);
        if (host.isEmpty()) {
            return false;
        }

        if ("localhost".equalsIgnoreCase(host)) {
            return true;
        }

        String[] parts = host.split("\\.");
        if (parts.length != 4) {
            return false;
        }

        int a;
        int b;
        try {
            a = Integer.parseInt(parts[0]);
            b = Integer.parseInt(parts[1]);
        } catch (NumberFormatException e) {
            return false;
        }

        if (a == 10) {
            return true;
        }

        if (a == 172 && b >= 16 && b <= 31) {
            return true;
        }

        return a == 192 && b == 168;
    }

    private String extractHost(String hostOrUrl) {
        if (hostOrUrl == null) {
            return "";
        }

        String raw = hostOrUrl.trim();
        if (raw.isEmpty()) {
            return "";
        }

        if (!raw.startsWith("http://") && !raw.startsWith("https://")) {
            raw = "http://" + raw;
        }

        HttpUrl url = HttpUrl.parse(raw);
        if (url == null || url.host() == null) {
            return "";
        }

        return url.host();
    }
}