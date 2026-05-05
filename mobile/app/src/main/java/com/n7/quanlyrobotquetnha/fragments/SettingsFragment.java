package com.n7.quanlyrobotquetnha.fragments;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Switch;
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

    private EditText edtSsid;
    private EditText edtPass;
    private EditText edtGoSpeed;
    private EditText edtTurnSpeed;
    private EditText edtWheelRadius;
    private EditText edtRotateReverseThreshold;
    private EditText edtRotateTolerance;
    private EditText edtRotateMaxCorrections;
    private EditText edtWallDistance;
    private EditText edtSleepMinutes;
    private Switch swPulseEnabled;
    private EditText edtPulseFreq;
    private EditText edtPulseDuty;
    private EditText edtPulsePower;
    private EditText edtZigzagAngleOffsetM;
    private EditText edtZigzagDeviationThresholdN;
    private EditText edtZigzagStraightDistanceK;
    private EditText edtZigzagOffsetDistanceL;
    private TextView tvConfigNote;
    private Button btnUpdateConfig;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_config, container, false);

        edtSsid = view.findViewById(R.id.edtCfgSsid);
        edtPass = view.findViewById(R.id.edtCfgPass);
        edtGoSpeed = view.findViewById(R.id.edtCfgGoSpeed);
        edtTurnSpeed = view.findViewById(R.id.edtCfgTurnSpeed);
        edtWheelRadius = view.findViewById(R.id.edtCfgWheelRadius);
        edtRotateReverseThreshold = view.findViewById(R.id.edtCfgRotateReverseThreshold);
        edtRotateTolerance = view.findViewById(R.id.edtCfgRotateTolerance);
        edtRotateMaxCorrections = view.findViewById(R.id.edtCfgRotateMaxCorrections);
        edtWallDistance = view.findViewById(R.id.edtCfgWallDistance);
        edtSleepMinutes = view.findViewById(R.id.edtCfgSleepMinutes);
        swPulseEnabled = view.findViewById(R.id.swPulseEnabled);
        edtPulseFreq = view.findViewById(R.id.edtCfgPulseFreq);
        edtPulseDuty = view.findViewById(R.id.edtCfgPulseDuty);
        edtPulsePower = view.findViewById(R.id.edtCfgPulsePower);
        edtZigzagAngleOffsetM = view.findViewById(R.id.edtCfgZigzagAngleOffsetM);
        edtZigzagDeviationThresholdN = view.findViewById(R.id.edtCfgZigzagDeviationThresholdN);
        edtZigzagStraightDistanceK = view.findViewById(R.id.edtCfgZigzagStraightDistanceK);
        edtZigzagOffsetDistanceL = view.findViewById(R.id.edtCfgZigzagOffsetDistanceL);
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

        String ssid = edtSsid.getText().toString().trim();
        String pass = edtPass.getText().toString().trim();

        int goSpeed;
        int turnSpeed;
        float wheelRadius;
        float rotateReverseThresholdDeg;
        float rotateToleranceDeg;
        int rotateMaxCorrections;
        float wallDistance;
        int sleepMinutes;
        boolean pulseEnabled;
        float pulseFreqHz;
        float pulseDutyPercent;
        float pulsePowerPercent;
        float zigzagAngleOffsetM;
        float zigzagDeviationThresholdN;
        float zigzagStraightDistanceK;
        float zigzagOffsetDistanceL;

        try {
            goSpeed = Integer.parseInt(edtGoSpeed.getText().toString().trim());
            turnSpeed = Integer.parseInt(edtTurnSpeed.getText().toString().trim());
            wheelRadius = Float.parseFloat(edtWheelRadius.getText().toString().trim());
            rotateReverseThresholdDeg = Float.parseFloat(edtRotateReverseThreshold.getText().toString().trim());
            rotateToleranceDeg = Float.parseFloat(edtRotateTolerance.getText().toString().trim());
            rotateMaxCorrections = Integer.parseInt(edtRotateMaxCorrections.getText().toString().trim());
            wallDistance = Float.parseFloat(edtWallDistance.getText().toString().trim());
            sleepMinutes = Integer.parseInt(edtSleepMinutes.getText().toString().trim());
            pulseEnabled = swPulseEnabled.isChecked();
            pulseFreqHz = Float.parseFloat(edtPulseFreq.getText().toString().trim());
            pulseDutyPercent = Float.parseFloat(edtPulseDuty.getText().toString().trim());
            pulsePowerPercent = Float.parseFloat(edtPulsePower.getText().toString().trim());
            zigzagAngleOffsetM = Float.parseFloat(edtZigzagAngleOffsetM.getText().toString().trim());
            zigzagDeviationThresholdN = Float.parseFloat(edtZigzagDeviationThresholdN.getText().toString().trim());
            zigzagStraightDistanceK = Float.parseFloat(edtZigzagStraightDistanceK.getText().toString().trim());
            zigzagOffsetDistanceL = Float.parseFloat(edtZigzagOffsetDistanceL.getText().toString().trim());
        } catch (Exception e) {
            Toast.makeText(requireContext(), "Thông số cấu hình không hợp lệ", Toast.LENGTH_SHORT).show();
            return;
        }

        JsonObject config = new JsonObject();
        config.addProperty("ssid", ssid);
        config.addProperty("pass", pass);
        config.addProperty("goSpeed", goSpeed);
        config.addProperty("turnSpeed", turnSpeed);
        config.addProperty("wheelRadiusCm", wheelRadius);
        config.addProperty("rotateReverseThresholdDeg", rotateReverseThresholdDeg);
        config.addProperty("rotateToleranceDeg", rotateToleranceDeg);
        config.addProperty("rotateMaxCorrections", rotateMaxCorrections);
        config.addProperty("wallStopDistanceCm", wallDistance);
        config.addProperty("sleepMinutes", sleepMinutes);
        config.addProperty("pulseEnabled", pulseEnabled);
        config.addProperty("pulseFreqHz", pulseFreqHz);
        config.addProperty("pulseDutyPercent", pulseDutyPercent);
        config.addProperty("pulsePowerPercent", pulsePowerPercent);
        config.addProperty("zigzagAngleOffsetM", zigzagAngleOffsetM);
        config.addProperty("zigzagDeviationThresholdN", zigzagDeviationThresholdN);
        config.addProperty("zigzagStraightDistanceK", zigzagStraightDistanceK);
        config.addProperty("zigzagOffsetDistanceL", zigzagOffsetDistanceL);

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

        edtSsid.setText(readString(config, "ssid", ""));
        edtPass.setText(readString(config, "pass", ""));
        edtGoSpeed.setText(String.valueOf(readInt(config, "goSpeed", 60)));
        edtTurnSpeed.setText(String.valueOf(readInt(config, "turnSpeed", 60)));
        edtWheelRadius.setText(String.valueOf(readFloat(config, "wheelRadiusCm", 3.4f)));
        edtRotateReverseThreshold.setText(String.valueOf(readFloat(config, "rotateReverseThresholdDeg", 8f)));
        edtRotateTolerance.setText(String.valueOf(readFloat(config, "rotateToleranceDeg", 2f)));
        edtRotateMaxCorrections.setText(String.valueOf(readInt(config, "rotateMaxCorrections", 2)));
        edtWallDistance.setText(String.valueOf(readFloat(config, "wallStopDistanceCm", 20f)));
        edtSleepMinutes.setText(String.valueOf(readInt(config, "sleepMinutes", 10)));
        swPulseEnabled.setChecked(readBoolean(config, "pulseEnabled", true));
        edtPulseFreq.setText(String.valueOf(readFloat(config, "pulseFreqHz", 12.5f)));
        edtPulseDuty.setText(String.valueOf(readFloat(config, "pulseDutyPercent", 50f)));
        edtPulsePower.setText(String.valueOf(readFloat(config, "pulsePowerPercent", 100f)));
        edtZigzagAngleOffsetM.setText(String.valueOf(readFloat(config, "zigzagAngleOffsetM", 28f)));
        edtZigzagDeviationThresholdN.setText(String.valueOf(readFloat(config, "zigzagDeviationThresholdN", 5f)));
        edtZigzagStraightDistanceK.setText(String.valueOf(readFloat(config, "zigzagStraightDistanceK", 100f)));
        edtZigzagOffsetDistanceL.setText(String.valueOf(readFloat(config, "zigzagOffsetDistanceL", 30f)));
    }

    private String readString(JsonObject obj, String key, String fallback) {
        if (!obj.has(key) || obj.get(key).isJsonNull()) {
            return fallback;
        }
        return obj.get(key).getAsString();
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

    private boolean readBoolean(JsonObject obj, String key, boolean fallback) {
        if (!obj.has(key) || obj.get(key).isJsonNull()) {
            return fallback;
        }

        try {
            return obj.get(key).getAsBoolean();
        } catch (Exception e) {
            return fallback;
        }
    }
}
