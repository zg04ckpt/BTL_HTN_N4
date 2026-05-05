package com.n7.quanlyrobotquetnha.activities;

import android.os.Bundle;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import com.google.gson.Gson;
import com.n7.quanlyrobotquetnha.R;
import com.n7.quanlyrobotquetnha.models.MoveInfo;
import com.n7.quanlyrobotquetnha.utils.HttpClient;
import com.n7.quanlyrobotquetnha.utils.PreferenceManager;

import java.io.IOException;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.Response;

public class ControlActivity extends AppCompatActivity {

    private static final float GO_DURATION_SECONDS = 2f;

    ImageView ivBack;
    EditText edtAngle;
    Button btnRotate;
    Button btnGo;
    private final Gson gson = new Gson();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control);

        String savedHost = PreferenceManager.getInstance(this)
                .getString(PreferenceManager.KEY_ROBOT_IP, "");
        if (!savedHost.isEmpty()) {
            HttpClient.setRobotHost(savedHost);
        }

        ivBack = findViewById(R.id.ivBack);
        edtAngle = findViewById(R.id.edtAngle);
        btnRotate = findViewById(R.id.btnRotate);
        btnGo = findViewById(R.id.btnGo);

        setListeners();
    }

    private void setListeners() {
        ivBack.setOnClickListener(v -> finish());
        btnRotate.setOnClickListener(v -> {
            String angle = edtAngle.getText().toString();
            if (angle.isEmpty()) return;

            btnRotate.setEnabled(false);
            float angleValue;
            try {
                angleValue = Float.parseFloat(angle);
            } catch (NumberFormatException ex) {
                btnRotate.setEnabled(true);
                Toast.makeText(ControlActivity.this, "Góc không hợp lệ", Toast.LENGTH_SHORT).show();
                return;
            }
            String json = gson.toJson(MoveInfo.rotate(angleValue));

            HttpClient.post("/control/move", json, new Callback() {
                @Override
                public void onFailure(@NonNull Call call, @NonNull IOException e) {
                    runOnUiThread(() -> {
                        btnRotate.setEnabled(true);
                        Toast.makeText(ControlActivity.this, "Gửi lệnh xoay thất bại", Toast.LENGTH_SHORT).show();
                    });
                }

                @Override
                public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                    runOnUiThread(() -> {
                        btnRotate.setEnabled(true);
                        if (response.isSuccessful()) {
                            Toast.makeText(ControlActivity.this, "Đã gửi lệnh xoay", Toast.LENGTH_SHORT).show();
                        } else {
                            Toast.makeText(ControlActivity.this, "Lỗi: " + response.code(), Toast.LENGTH_SHORT).show();
                        }
                    });
                }
            });
        });

        btnGo.setOnClickListener(l -> {
            btnGo.setEnabled(false);
            String json = gson.toJson(MoveInfo.go(GO_DURATION_SECONDS));

            HttpClient.post("/control/move", json, new Callback() {
                @Override
                public void onFailure(@NonNull Call call, @NonNull IOException e) {
                    runOnUiThread(() -> {
                        Toast.makeText(ControlActivity.this, "Gửi lệnh tiến thất bại", Toast.LENGTH_SHORT).show();
                        btnGo.setEnabled(true);
                    });
                }

                @Override
                public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                    runOnUiThread(() -> {
                        btnGo.setEnabled(true);
                        if (response.isSuccessful()) {
                            Toast.makeText(ControlActivity.this, "Đã gửi lệnh tiến", Toast.LENGTH_SHORT).show();
                        } else {
                            Toast.makeText(ControlActivity.this, "Lỗi: " + response.code(), Toast.LENGTH_SHORT).show();
                        }
                    });
                }
            });
        });
    }
}
