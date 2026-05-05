package com.n7.quanlyrobotquetnha.fragments;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.n7.quanlyrobotquetnha.R;
import com.n7.quanlyrobotquetnha.utils.RobotSocketManager;

public class SettingsFragment extends Fragment implements RobotSocketManager.Listener {

    private EditText edtBasePwm;
    private EditText edtLeftGain;
    private EditText edtRightGain;
    private EditText edtWallDistance;
    private EditText edtEndDistance;
    private EditText edtMinStartPwmL;
    private EditText edtMinStartPwmR;
    private TextView tvConfigNote;
    private Button btnUpdateConfig;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_config, container, false);

        edtBasePwm = view.findViewById(R.id.edtCfgBasePwm);
        edtLeftGain = view.findViewById(R.id.edtCfgLeftGain);
        edtRightGain = view.findViewById(R.id.edtCfgRightGain);
        edtWallDistance = view.findViewById(R.id.edtCfgWallDistance);
        edtEndDistance = view.findViewById(R.id.edtCfgEndDistance);
        edtMinStartPwmL = view.findViewById(R.id.edtCfgMinStartPwmL);
        edtMinStartPwmR = view.findViewById(R.id.edtCfgMinStartPwmR);
        tvConfigNote = view.findViewById(R.id.tvConfigNote);
        btnUpdateConfig = view.findViewById(R.id.btnUpdateConfig);

        btnUpdateConfig.setOnClickListener(v -> sendConfig());
        return view;
    }

    @Override
    public void onStart() {
        super.onStart();
        RobotSocketManager.getInstance().addListener(this);
        requestConfig();
    }

    @Override
    public void onStop() {
        RobotSocketManager.getInstance().removeListener(this);
        super.onStop();
    }

    @Override
    public void onSocketConnected() {
        requestConfig();
    }

    @Override
    public void onSocketMessage(String text) {
        try {
            JsonObject root = JsonParser.parseString(text).getAsJsonObject();
            String type = root.has("type") ? root.get("type").getAsString() : "";

            if ("config".equals(type)) {
                bindConfig(root.getAsJsonObject("config"));
                return;
            }

            if ("ack".equals(type) && root.has("action")
                    && "update_config".equals(root.get("action").getAsString())
                    && isAdded()) {
                String msg = root.has("message") ? root.get("message").getAsString() : "Đã gửi cập nhật cấu hình";
                Toast.makeText(requireContext(), msg, Toast.LENGTH_SHORT).show();
            }
        } catch (Exception ignored) {
        }
    }

    private void sendConfig() {
        if (!isAdded()) {
            return;
        }

        int basePwm;
        float leftGain;
        float rightGain;
        float wallDistance;
        float endDistance;
        int minStartPwmL;
        int minStartPwmR;

        try {
            basePwm = Integer.parseInt(edtBasePwm.getText().toString().trim());
            leftGain = Float.parseFloat(edtLeftGain.getText().toString().trim());
            rightGain = Float.parseFloat(edtRightGain.getText().toString().trim());
            wallDistance = Float.parseFloat(edtWallDistance.getText().toString().trim());
            endDistance = Float.parseFloat(edtEndDistance.getText().toString().trim());
            minStartPwmL = Integer.parseInt(edtMinStartPwmL.getText().toString().trim());
            minStartPwmR = Integer.parseInt(edtMinStartPwmR.getText().toString().trim());
        } catch (Exception e) {
            Toast.makeText(requireContext(), "Thông số cấu hình không hợp lệ", Toast.LENGTH_SHORT).show();
            return;
        }

        JsonObject config = new JsonObject();
        config.addProperty("basePwm", basePwm);
        config.addProperty("leftGain", leftGain);
        config.addProperty("rightGain", rightGain);
        config.addProperty("wallDistanceCm", wallDistance);
        config.addProperty("endDistanceCm", endDistance);
        config.addProperty("minStartPwmL", minStartPwmL);
        config.addProperty("minStartPwmR", minStartPwmR);

        JsonObject payload = new JsonObject();
        payload.addProperty("type", "update_config");
        payload.add("config", config);

        boolean sent = RobotSocketManager.getInstance().sendJson(payload);
        if (!sent) {
            Toast.makeText(requireContext(), "Socket chưa kết nối", Toast.LENGTH_SHORT).show();
            return;
        }

        tvConfigNote.setText("Đã gửi cấu hình. Robot sẽ áp dụng ở chu kỳ làm việc tiếp theo.");
    }

    private void requestConfig() {
        JsonObject payload = new JsonObject();
        payload.addProperty("type", "get_config");
        RobotSocketManager.getInstance().sendJson(payload);
    }

    private void bindConfig(JsonObject config) {
        if (!isAdded() || config == null) {
            return;
        }

        edtBasePwm.setText(String.valueOf(readInt(config, "basePwm", readInt(config, "goSpeed", 85))));
        edtLeftGain.setText(String.valueOf(readFloat(config, "leftGain", 1.0f)));
        edtRightGain.setText(String.valueOf(readFloat(config, "rightGain", 1.0f)));
        edtWallDistance.setText(String.valueOf(readFloat(config, "wallDistanceCm", readFloat(config, "wallStopDistanceCm", 20f))));
        edtEndDistance.setText(String.valueOf(readFloat(config, "endDistanceCm", 10f)));
        edtMinStartPwmL.setText(String.valueOf(readInt(config, "minStartPwmL", 50)));
        edtMinStartPwmR.setText(String.valueOf(readInt(config, "minStartPwmR", 50)));
    }

    private int readInt(JsonObject obj, String key, int fallback) {
        if (!obj.has(key) || obj.get(key).isJsonNull()) {
            return fallback;
        }

        try {
            return obj.get(key).getAsInt();
        } catch (Exception e) {
            return fallback;
        }
    }

    private float readFloat(JsonObject obj, String key, float fallback) {
        if (!obj.has(key) || obj.get(key).isJsonNull()) {
            return fallback;
        }

        try {
            return obj.get(key).getAsFloat();
        } catch (Exception e) {
            return fallback;
        }
    }
}
