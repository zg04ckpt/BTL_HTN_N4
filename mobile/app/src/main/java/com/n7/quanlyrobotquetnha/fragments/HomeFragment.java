package com.n7.quanlyrobotquetnha.fragments;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.n7.quanlyrobotquetnha.R;
import com.n7.quanlyrobotquetnha.utils.RobotSocketManager;

public class HomeFragment extends Fragment implements RobotSocketManager.Listener {

    private TextView tvStatusCode;
    private TextView tvStatusDesc;
    private TextView tvPowerState;
    private Button btnPowerOn;
    private Button btnPowerOff;
    private Button btnStop;
    private Button btnGoHome;
    private Button btnWorkNow;

    private String currentStatus = "UNKNOWN";
    private String currentPower = "ON";

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_status, container, false);

        tvStatusCode = view.findViewById(R.id.tvStatusCode);
        tvStatusDesc = view.findViewById(R.id.tvStatusDesc);
        tvPowerState = view.findViewById(R.id.tvPowerState);
        btnPowerOn = view.findViewById(R.id.btnPowerOn);
        btnPowerOff = view.findViewById(R.id.btnPowerOff);
        btnStop = view.findViewById(R.id.btnStop);
        btnGoHome = view.findViewById(R.id.btnGoHome);
        btnWorkNow = view.findViewById(R.id.btnWorkNow);

        btnPowerOn.setOnClickListener(v -> sendCommand("power_on"));
        btnPowerOff.setOnClickListener(v -> sendCommand("power_off"));
        btnStop.setOnClickListener(v -> sendCommand("stop"));
        btnGoHome.setOnClickListener(v -> sendCommand("go_home"));
        btnWorkNow.setOnClickListener(v -> sendCommand("start_work"));

        applyButtonState();
        return view;
    }

    @Override
    public void onStart() {
        super.onStart();
        RobotSocketManager.getInstance().addListener(this);
        requestStatus();
    }

    @Override
    public void onStop() {
        RobotSocketManager.getInstance().removeListener(this);
        super.onStop();
    }

    @Override
    public void onSocketConnected() {
        requestStatus();
    }

    @Override
    public void onSocketMessage(String text) {
        try {
            JsonObject root = JsonParser.parseString(text).getAsJsonObject();
            String type = root.has("type") ? root.get("type").getAsString() : "";
            if (!"status".equals(type) && !"connected".equals(type)) {
                return;
            }

            boolean remoteOn = true;
            if (root.has("remoteEnabled")) {
                remoteOn = root.get("remoteEnabled").getAsBoolean();
            }

            String nextStatus = root.has("status") ? root.get("status").getAsString() : "UNKNOWN";
            if (!remoteOn) {
                nextStatus = "OFF";
            }

            String statusText = root.has("statusText") ? root.get("statusText").getAsString() : "Không có mô tả";
            String nextPower = remoteOn ? "ON" : "OFF";

            updateStatusUi(nextStatus, statusText, nextPower);
        } catch (Exception ignored) {
        }
    }

    private void sendCommand(String command) {
        JsonObject payload = new JsonObject();
        payload.addProperty("type", "command");
        payload.addProperty("command", command);

        boolean sent = RobotSocketManager.getInstance().sendJson(payload);
        if (!sent && isAdded()) {
            Toast.makeText(requireContext(), "Socket chưa kết nối", Toast.LENGTH_SHORT).show();
        }
    }

    private void requestStatus() {
        JsonObject payload = new JsonObject();
        payload.addProperty("type", "get_status");
        RobotSocketManager.getInstance().sendJson(payload);
    }

    private void updateStatusUi(String status, String description, String powerState) {
        if (!isAdded()) {
            return;
        }

        currentStatus = status;
        currentPower = powerState;
        tvStatusCode.setText("Mã trạng thái: " + status);
        tvStatusDesc.setText("Mô tả: " + description);
        tvPowerState.setText("Nguồn: " + powerState);
        applyButtonState();
    }

    private void applyButtonState() {
        boolean connected = RobotSocketManager.getInstance().isConnected();

        if (!connected) {
            btnPowerOn.setEnabled(false);
            btnPowerOff.setEnabled(false);
            btnStop.setEnabled(false);
            btnGoHome.setEnabled(false);
            btnWorkNow.setEnabled(false);
            return;
        }

        boolean isOff = "OFF".equals(currentStatus) || "OFF".equals(currentPower);
        btnPowerOn.setEnabled(isOff);
        btnPowerOff.setEnabled(!isOff);

        if (isOff) {
            btnStop.setEnabled(false);
            btnGoHome.setEnabled(false);
            btnWorkNow.setEnabled(false);
            return;
        }

        boolean isRunning = "RUNNING".equals(currentStatus) || "BEGIN_JOB".equals(currentStatus);
        boolean canWorkNow = "SLEEPING".equals(currentStatus) || "WAIT_CONNECT".equals(currentStatus);
        boolean canGoHome = isRunning || "SLEEPING".equals(currentStatus);

        btnStop.setEnabled(isRunning);
        btnGoHome.setEnabled(canGoHome);
        btnWorkNow.setEnabled(canWorkNow);
    }
}
