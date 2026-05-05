package com.n7.quanlyrobotquetnha;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.provider.Settings;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.swiperefreshlayout.widget.SwipeRefreshLayout;

import com.n7.quanlyrobotquetnha.activities.ConfigActivity;
import com.n7.quanlyrobotquetnha.activities.ManageActivity;
import com.n7.quanlyrobotquetnha.utils.HttpClient;
import com.n7.quanlyrobotquetnha.utils.PreferenceManager;
import com.n7.quanlyrobotquetnha.utils.RobotUdpDiscovery;
import com.n7.quanlyrobotquetnha.utils.RobotSocketManager;
import com.n7.quanlyrobotquetnha.utils.WifiScanner;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.HttpUrl;
import okhttp3.Response;

public class MainActivity extends AppCompatActivity {

    private static final int REQ_WIFI_PERMISSIONS = 1101;
    private static final String ROBOT_AP_PREFIX = "ROBOT";
    private static final String ROBOT_AP_DEFAULT_HOST = "192.168.4.1";

    private TextView tvStatus;
    private Button btnAction;
    private EditText edtRobotHost;
    private Button btnConnectByHost;
    private Button btnClearWifiCache;
    private SwipeRefreshLayout swipeRefreshMain;
    private RobotUdpDiscovery robotUdpDiscovery;
    private String discoveredRobotIp = "";
    private PrimaryAction primaryAction = PrimaryAction.NONE;
    private boolean startupChecksRunning = false;

    private enum PrimaryAction {
        NONE,
        CONNECT_ROBOT,
        OPEN_CONFIG,
        OPEN_WIFI_SETTINGS,
        RETRY_CHECKS
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        tvStatus = findViewById(R.id.tvStatus);
        btnAction = findViewById(R.id.btnAction);
        edtRobotHost = findViewById(R.id.edtRobotHost);
        btnConnectByHost = findViewById(R.id.btnConnectByHost);
        btnClearWifiCache = findViewById(R.id.btnClearWifiCache);
        swipeRefreshMain = findViewById(R.id.swipeRefreshMain);

        setupInitialHost();
        setupListeners();
    }

    @Override
    protected void onResume() {
        super.onResume();
        runStartupChecks(false);
    }

    private void setupInitialHost() {
        String savedHost = PreferenceManager.getInstance(this)
                .getString(PreferenceManager.KEY_ROBOT_IP, "");

        if (!savedHost.isEmpty()) {
            edtRobotHost.setText(savedHost);
            if (HttpClient.setRobotHost(savedHost)) {
                tvStatus.setText("Đã nạp địa chỉ robot đã lưu: " + HttpClient.getRobotBaseUrl());
            }
        }
    }

    private void setupListeners() {
        btnAction.setOnClickListener(v -> handlePrimaryAction());
        btnConnectByHost.setOnClickListener(v -> connectByHost());
        btnClearWifiCache.setOnClickListener(v -> confirmClearLocalWifiCache());
        swipeRefreshMain.setOnRefreshListener(() -> runStartupChecks(true));
    }

    private void confirmClearLocalWifiCache() {
        new AlertDialog.Builder(this)
                .setTitle("Xóa bộ nhớ trên điện thoại")
                .setMessage("Xóa địa chỉ robot đã lưu và tên Wi‑Fi đã cấu hình gần nhất. Cấu hình Wi‑Fi trên robot không bị đổi.")
                .setNegativeButton("Hủy", null)
                .setPositiveButton("Xóa", (dialog, which) -> clearLocalWifiCache())
                .show();
    }

    private void clearLocalWifiCache() {
        PreferenceManager.getInstance(this).clearRobotWifiCache();
        RobotSocketManager.getInstance().disconnect("clear_wifi_cache");

        String gw = WifiScanner.getWifiGatewayHost(this);
        String nextHost = !gw.isEmpty() ? gw : ROBOT_AP_DEFAULT_HOST;
        if (HttpClient.setRobotHost(nextHost)) {
            edtRobotHost.setText(nextHost);
        } else {
            edtRobotHost.setText("");
        }

        tvStatus.setText("Đã xóa bộ nhớ kết nối. Nhập IP robot hoặc quét lại.");
        Toast.makeText(this, "Đã xóa bộ nhớ kết nối trên điện thoại.", Toast.LENGTH_SHORT).show();
    }

    private void runStartupChecks(boolean forceRestart) {
        if (startupChecksRunning && !forceRestart) {
            return;
        }

        if (robotUdpDiscovery != null) {
            robotUdpDiscovery.stopDiscovery();
        }

        discoveredRobotIp = "";
        startupChecksRunning = true;
        swipeRefreshMain.setRefreshing(true);
        setPrimaryAction(PrimaryAction.NONE, "");
        startLanDiscovery();
    }

    private void stopRefreshIndicator() {
        swipeRefreshMain.setRefreshing(false);
    }

    private void startLanDiscovery() {
        tvStatus.setText("Đang dò robot trong mạng nội bộ...");

        if (robotUdpDiscovery != null) {
            robotUdpDiscovery.stopDiscovery();
        }

        robotUdpDiscovery = new RobotUdpDiscovery(this, new RobotUdpDiscovery.DiscoveryCallback() {
            @Override
            public void onRobotFound(String ip) {
                runOnUiThread(() -> {
                    discoveredRobotIp = ip;
                    startupChecksRunning = false;
                    stopRefreshIndicator();
                    tvStatus.setText("Đã tìm thấy robot qua broadcast tại " + ip + ". Nhấn nút bên dưới để kết nối.");
                    setPrimaryAction(PrimaryAction.CONNECT_ROBOT, "Kết nối robot");
                });
            }

            @Override
            public void onError(String message) {
                runOnUiThread(() -> {
                    checkRobotApMode();
                });
            }
        });

        robotUdpDiscovery.startDiscovery();
    }

    private void checkRobotApMode() {
        tvStatus.setText("Không thấy robot qua broadcast LAN. Đang kiểm tra AP của robot...");

        if (!WifiScanner.isWifiEnabled(this)) {
            startupChecksRunning = false;
            stopRefreshIndicator();
            tvStatus.setText("Không thấy robot trong LAN. Hãy bật Wi-Fi để quét AP robot.");
            setPrimaryAction(PrimaryAction.OPEN_WIFI_SETTINGS, "Mở cài đặt Wi-Fi");
            return;
        }

        if (!hasWifiScanPermissions()) {
            requestWifiPermissions();
            return;
        }

        WifiScanner.checkNotConfigStatus(this, ROBOT_AP_PREFIX, (found, ssid) -> runOnUiThread(() -> {
            startupChecksRunning = false;
            stopRefreshIndicator();

            if (found) {
                tvStatus.setText("Phát hiện robot đang bật AP" + (ssid.isEmpty() ? "" : " (" + ssid + ")") + ". Nhấn Cấu hình để thiết lập Wi-Fi cho robot.");
                setPrimaryAction(PrimaryAction.OPEN_CONFIG, "Cấu hình robot");
                return;
            }

            checkLastConfiguredSsid();
        }));
    }

    private void checkLastConfiguredSsid() {
        String lastSsid = PreferenceManager.getInstance(this)
                .getString(PreferenceManager.KEY_LAST_CONFIG_WIFI_SSID, "")
                .trim();

        if (!lastSsid.isEmpty()) {
            String currentSsid = WifiScanner.getCurSSID(this);
            if (lastSsid.equals(currentSsid)) {
                tvStatus.setText("Bạn đang ở đúng mạng " + lastSsid + " nhưng chưa thấy robot phản hồi. Robot có thể chưa bật hoặc đang lỗi, vui lòng kiểm tra lại.");
            } else {
                tvStatus.setText("Lần cấu hình gần nhất robot dùng Wi-Fi " + lastSsid + ". Hãy chuyển sang mạng này rồi quét lại. Robot cũng có thể chưa bật hoặc đang lỗi.");
            }
        } else {
            tvStatus.setText("Không thấy robot qua LAN, cũng không thấy AP robot và chưa có lịch sử SSID. Robot có thể chưa bật hoặc bị lỗi, vui lòng kiểm tra lại.");
        }

        setPrimaryAction(PrimaryAction.RETRY_CHECKS, "Quét lại");
    }

    private void setPrimaryAction(PrimaryAction action, String text) {
        primaryAction = action;

        if (action == PrimaryAction.NONE || text == null || text.trim().isEmpty()) {
            btnAction.setVisibility(View.GONE);
            btnAction.setEnabled(false);
            return;
        }

        btnAction.setVisibility(View.VISIBLE);
        btnAction.setEnabled(true);
        btnAction.setText(text);
    }

    private void handlePrimaryAction() {
        switch (primaryAction) {
            case CONNECT_ROBOT:
                if (discoveredRobotIp.isEmpty()) {
                    Toast.makeText(this, "Không có thông tin robot để kết nối", Toast.LENGTH_SHORT).show();
                    return;
                }
                onRobotConnected(discoveredRobotIp, "Đã kết nối robot qua LAN");
                break;
            case OPEN_CONFIG:
                Intent configIntent = new Intent(this, ConfigActivity.class);
                configIntent.putExtra(ConfigActivity.EXTRA_FORCE_ROBOT_HOST, ROBOT_AP_DEFAULT_HOST);
                startActivity(configIntent);
                break;
            case OPEN_WIFI_SETTINGS:
                openWifiSettings();
                break;
            case RETRY_CHECKS:
                runStartupChecks(true);
                break;
            case NONE:
            default:
                break;
        }
    }

    private void openWifiSettings() {
        try {
            Intent intent;
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                intent = new Intent(Settings.Panel.ACTION_WIFI);
            } else {
                intent = new Intent(Settings.ACTION_WIFI_SETTINGS);
            }
            startActivity(intent);
            setPrimaryAction(PrimaryAction.RETRY_CHECKS, "Tôi đã bật Wi-Fi, quét lại");
        } catch (Exception e) {
            Toast.makeText(this, "Không thể mở cài đặt Wi-Fi", Toast.LENGTH_SHORT).show();
            setPrimaryAction(PrimaryAction.RETRY_CHECKS, "Quét lại");
        }
    }

    private boolean hasWifiScanPermissions() {
        boolean fineLocationGranted = ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED;

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            boolean nearbyWifiGranted = ContextCompat.checkSelfPermission(this, Manifest.permission.NEARBY_WIFI_DEVICES)
                    == PackageManager.PERMISSION_GRANTED;
            return fineLocationGranted && nearbyWifiGranted;
        }

        return fineLocationGranted;
    }

    private void requestWifiPermissions() {
        List<String> permissions = new ArrayList<>();
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            permissions.add(Manifest.permission.ACCESS_FINE_LOCATION);
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU
                && ContextCompat.checkSelfPermission(this, Manifest.permission.NEARBY_WIFI_DEVICES)
                != PackageManager.PERMISSION_GRANTED) {
            permissions.add(Manifest.permission.NEARBY_WIFI_DEVICES);
        }

        if (permissions.isEmpty()) {
            checkRobotApMode();
            return;
        }

        tvStatus.setText("Cần quyền truy cập vị trí/Wi-Fi để kiểm tra AP robot.");
        setPrimaryAction(PrimaryAction.NONE, "");
        ActivityCompat.requestPermissions(this, permissions.toArray(new String[0]), REQ_WIFI_PERMISSIONS);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode != REQ_WIFI_PERMISSIONS) {
            return;
        }

        boolean allGranted = true;
        for (int result : grantResults) {
            if (result != PackageManager.PERMISSION_GRANTED) {
                allGranted = false;
                break;
            }
        }

        if (allGranted) {
            checkRobotApMode();
            return;
        }

        startupChecksRunning = false;
        stopRefreshIndicator();
        tvStatus.setText("Không đủ quyền để kiểm tra AP robot. Hãy cấp quyền rồi thử quét lại.");
        setPrimaryAction(PrimaryAction.RETRY_CHECKS, "Thử lại");
    }

    private void connectByHost() {
        String hostInput = edtRobotHost.getText().toString().trim();
        if (hostInput.isEmpty()) {
            Toast.makeText(this, "Vui lòng nhập IP/domain robot", Toast.LENGTH_SHORT).show();
            return;
        }

        btnConnectByHost.setEnabled(false);
        tvStatus.setText("Đang kiểm tra kết nối robot...");

        try {
            HttpClient.getWithBaseUrl(hostInput, "/status", new Callback() {
                @Override
                public void onFailure(@NonNull Call call, @NonNull IOException e) {
                    runOnUiThread(() -> {
                        btnConnectByHost.setEnabled(true);
                        tvStatus.setText("Không thể kết nối robot qua địa chỉ đã nhập");
                        Toast.makeText(MainActivity.this, "Lỗi kết nối: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                    });
                }

                @Override
                public void onResponse(@NonNull Call call, @NonNull Response response) {
                    int statusCode = response.code();
                    response.close();

                    runOnUiThread(() -> {
                        btnConnectByHost.setEnabled(true);
                        if (statusCode >= 200 && statusCode < 300) {
                            onRobotConnected(hostInput, "Kết nối robot thành công");
                        } else {
                            tvStatus.setText("Kết nối thất bại, robot phản hồi mã: " + statusCode);
                            Toast.makeText(MainActivity.this, "Địa chỉ có phản hồi nhưng không hợp lệ để kết nối", Toast.LENGTH_SHORT).show();
                        }
                    });
                }
            });
        } catch (IllegalArgumentException e) {
            btnConnectByHost.setEnabled(true);
            tvStatus.setText("Địa chỉ robot không hợp lệ");
            Toast.makeText(this, e.getMessage(), Toast.LENGTH_SHORT).show();
        }
    }

    private void applyRobotHost(String host, String prefix) {
        if (!HttpClient.setRobotHost(host)) {
            tvStatus.setText("Địa chỉ robot không hợp lệ");
            return;
        }

        PreferenceManager.getInstance(this).putString(PreferenceManager.KEY_ROBOT_IP, host);
        edtRobotHost.setText(host);
        tvStatus.setText(prefix + ": " + HttpClient.getRobotBaseUrl());
        Toast.makeText(this, "Đã lưu địa chỉ robot", Toast.LENGTH_SHORT).show();
    }

    private void onRobotConnected(String host, String statusPrefix) {
        if (!isLanHost(host)) {
            tvStatus.setText("Socket chỉ hỗ trợ mạng nội bộ. Vui lòng quét robot trong LAN.");
            Toast.makeText(this, "Địa chỉ không thuộc mạng nội bộ", Toast.LENGTH_SHORT).show();
            return;
        }

        applyRobotHost(host, statusPrefix);

        btnAction.setEnabled(false);
        btnConnectByHost.setEnabled(false);
        tvStatus.setText("Đang thiết lập kết nối socket với robot...");

        RobotSocketManager.getInstance().connect(HttpClient.getRobotBaseUrl(), new RobotSocketManager.ConnectCallback() {
            @Override
            public void onConnected() {
                runOnUiThread(() -> {
                    btnAction.setEnabled(true);
                    btnConnectByHost.setEnabled(true);
                    tvStatus.setText("Socket đã kết nối. Đang chuyển đến trang quản lý.");
                    startActivity(new Intent(MainActivity.this, ManageActivity.class));
                });
            }

            @Override
            public void onFailure(String reason) {
                runOnUiThread(() -> {
                    btnAction.setEnabled(true);
                    btnConnectByHost.setEnabled(true);
                    tvStatus.setText("Kết nối socket thất bại. Vui lòng thử lại.");
                    Toast.makeText(MainActivity.this, "Socket lỗi: " + reason, Toast.LENGTH_SHORT).show();
                });
            }
        });
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

    @Override
    protected void onDestroy() {
        if (robotUdpDiscovery != null) {
            robotUdpDiscovery.stopDiscovery();
        }
        super.onDestroy();
    }
}
