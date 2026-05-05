package com.n7.quanlyrobotquetnha.activities;

import android.os.Bundle;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import com.google.gson.Gson;
import com.n7.quanlyrobotquetnha.R;
import com.n7.quanlyrobotquetnha.models.WifiConfig;
import com.n7.quanlyrobotquetnha.utils.HttpClient;
import com.n7.quanlyrobotquetnha.utils.PreferenceManager;
import com.n7.quanlyrobotquetnha.utils.RobotSocketManager;
import com.n7.quanlyrobotquetnha.utils.WifiScanner;

import java.io.IOException;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.Response;

public class ConfigActivity extends AppCompatActivity {

    public static final String EXTRA_FORCE_ROBOT_HOST = "extra_force_robot_host";
    private static final String DEFAULT_AP_HOST = "192.168.4.1";
    private static final String ROBOT_AP_PREFIX = "robot";
    private static final String BUTTON_TEXT_DEFAULT = "Lưu cấu hình";
    private static final String BUTTON_TEXT_LOADING = "Đang cấu hình...";

    EditText edtSSID;
    EditText edtPass;
    Button btnSaveConfig;
    Button btnClearWifiCache;
    ImageView ivBack;

    private String savedHost = "";
    private String preferredHost = "";
    private String resolvedRobotHost = "";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_config);

        savedHost = PreferenceManager.getInstance(this)
                .getString(PreferenceManager.KEY_ROBOT_IP, "");

        preferredHost = getIntent().getStringExtra(EXTRA_FORCE_ROBOT_HOST);
        if (preferredHost != null) {
            preferredHost = preferredHost.trim();
        } else {
            preferredHost = "";
        }

        resolvedRobotHost = resolveRobotHost();
        if (!resolvedRobotHost.isEmpty()) {
            HttpClient.setRobotHost(resolvedRobotHost);
        }

        edtSSID = findViewById(R.id.edtSSID);
        edtPass = findViewById(R.id.edtPass);
        btnSaveConfig = findViewById(R.id.btnSaveConfig);
        btnClearWifiCache = findViewById(R.id.btnClearWifiCache);
        ivBack = findViewById(R.id.ivBack);

        setListeners();
    }

    private void setListeners() {
        ivBack.setOnClickListener(v -> finish());
        btnClearWifiCache.setOnClickListener(v -> confirmClearLocalWifiCache());
        btnSaveConfig.setOnClickListener(v -> {
            String ssid = edtSSID.getText().toString();
            String pass = edtPass.getText().toString();

            if (ssid.isEmpty() || pass.isEmpty()) {
                Toast.makeText(this, "Vui lòng nhập đầy đủ thông tin", Toast.LENGTH_SHORT).show();
                return;
            }

            // Chuyển đối tượng WifiConfig thành chuỗi JSON
            String jsonConfig = new Gson().toJson(new WifiConfig(ssid, pass));
            setLoadingState(true);

            if (!HttpClient.setRobotHost(resolvedRobotHost)) {
                setLoadingState(false);
                Toast.makeText(ConfigActivity.this, "Địa chỉ robot không hợp lệ", Toast.LENGTH_SHORT).show();
                return;
            }

            postWifiConfig(jsonConfig, ssid);
        });
    }

    private void confirmClearLocalWifiCache() {
        new AlertDialog.Builder(this)
                .setTitle("Xóa bộ nhớ trên điện thoại")
                .setMessage("Xóa địa chỉ robot đã lưu và tên Wi‑Fi đã cấu hình gần nhất (gợi ý kết nối). Cấu hình Wi‑Fi trên robot không bị đổi.")
                .setNegativeButton("Hủy", null)
                .setPositiveButton("Xóa", (dialog, which) -> clearLocalWifiCache())
                .show();
    }

    private void clearLocalWifiCache() {
        PreferenceManager.getInstance(this).clearRobotWifiCache();
        savedHost = "";
        resolvedRobotHost = resolveRobotHost();
        if (!resolvedRobotHost.isEmpty()) {
            HttpClient.setRobotHost(resolvedRobotHost);
        }
        RobotSocketManager.getInstance().disconnect("clear_wifi_cache");
        Toast.makeText(this, "Đã xóa bộ nhớ kết nối trên điện thoại.", Toast.LENGTH_SHORT).show();
    }

    private String resolveRobotHost() {
        if (!preferredHost.isEmpty()) {
            return preferredHost;
        }

        String currentSsid = WifiScanner.getCurSSID(this).toLowerCase();
        String gatewayHost = WifiScanner.getWifiGatewayHost(this);

        // Root cause của timeout: đang ở AP robot nhưng request lại đi vào host cũ.
        if (!gatewayHost.isEmpty() && currentSsid.startsWith(ROBOT_AP_PREFIX)) {
            return gatewayHost;
        }

        if (!savedHost.isEmpty()) {
            return savedHost;
        }

        if (!gatewayHost.isEmpty()) {
            return gatewayHost;
        }

        return DEFAULT_AP_HOST;
    }

    private void postWifiConfig(String jsonConfig, String ssid) {
        HttpClient.post("/config/wifi", jsonConfig, new Callback() {
            @Override
            public void onFailure(@NonNull Call call, @NonNull IOException e) {
                runOnUiThread(() -> {
                    setLoadingState(false);
                    Toast.makeText(ConfigActivity.this,
                            "Cấu hình thất bại (" + resolvedRobotHost + "): " + e.getMessage(),
                            Toast.LENGTH_SHORT).show();
                });
            }

            @Override
            public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                int statusCode = response.code();
                response.close();

                if (statusCode >= 200 && statusCode < 300) {
                    PreferenceManager.getInstance(ConfigActivity.this).putString(PreferenceManager.KEY_LAST_CONFIG_WIFI_SSID, ssid);
                    runOnUiThread(() -> {
                        setLoadingState(false);
                        Toast.makeText(ConfigActivity.this, "Cấu hình wifi thành công", Toast.LENGTH_SHORT).show();
                        finish();
                    });
                    return;
                }

                runOnUiThread(() -> {
                    setLoadingState(false);
                    Toast.makeText(ConfigActivity.this,
                            "Robot từ chối cấu hình (Mã lỗi: " + statusCode + ")",
                            Toast.LENGTH_SHORT).show();
                });
            }
        });
    }

    private void setLoadingState(boolean isLoading) {
        btnSaveConfig.setEnabled(!isLoading);
        btnSaveConfig.setText(isLoading ? BUTTON_TEXT_LOADING : BUTTON_TEXT_DEFAULT);
    }
}
