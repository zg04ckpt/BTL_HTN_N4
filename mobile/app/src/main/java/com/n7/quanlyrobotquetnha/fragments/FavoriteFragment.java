package com.n7.quanlyrobotquetnha.fragments;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import com.google.gson.JsonObject;
import com.n7.quanlyrobotquetnha.R;
import com.n7.quanlyrobotquetnha.utils.RobotSocketManager;

public class FavoriteFragment extends Fragment {

    private EditText edtGoDistance;
    private EditText edtRotateAngle;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_debug, container, false);

        edtGoDistance = view.findViewById(R.id.edtGoDistance);
        edtRotateAngle = view.findViewById(R.id.edtRotateAngle);
        Button btnDebugGo = view.findViewById(R.id.btnDebugGo);
        Button btnDebugRotate = view.findViewById(R.id.btnDebugRotate);

        btnDebugGo.setOnClickListener(v -> sendDebugMove("go", edtGoDistance.getText().toString().trim()));
        btnDebugRotate.setOnClickListener(v -> sendDebugMove("rotate", edtRotateAngle.getText().toString().trim()));

        return view;
    }

    private void sendDebugMove(String moveType, String valueText) {
        if (!isAdded()) {
            return;
        }

        float value;
        try {
            value = Float.parseFloat(valueText);
        } catch (Exception e) {
            Toast.makeText(requireContext(), "Giá trị lệnh không hợp lệ", Toast.LENGTH_SHORT).show();
            return;
        }

        JsonObject payload = new JsonObject();
        payload.addProperty("type", "debug_move");
        payload.addProperty("moveType", moveType);
        payload.addProperty("value", value);

        boolean sent = RobotSocketManager.getInstance().sendJson(payload);
        if (!sent) {
            Toast.makeText(requireContext(), "Socket chưa kết nối", Toast.LENGTH_SHORT).show();
            return;
        }

        Toast.makeText(requireContext(), "Đã gửi lệnh debug", Toast.LENGTH_SHORT).show();
    }
}
