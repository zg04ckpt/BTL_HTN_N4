package com.n7.quanlyrobotquetnha.utils;

import android.content.Context;
import android.util.Log;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class RobotUdpDiscovery {
    private static final String TAG = "RobotUDP";
    private static final int PORT = 4210; // Phải trùng với port trên ESP32
    private static final String REQUEST_MSG = "ROBOT_DISCOVER";

    private boolean isScanning = false;
    private DiscoveryCallback callback;
    private Context context;

    public interface DiscoveryCallback {
        void onRobotFound(String ip);
        void onError(String message);
    }

    public RobotUdpDiscovery(Context context, DiscoveryCallback callback) {
        this.context = context;
        this.callback = callback;
    }

    public void startDiscovery() {
        if (isScanning) return;
        isScanning = true;

        new Thread(() -> {
            DatagramSocket socket = null;
            try {
                socket = new DatagramSocket();
                socket.setBroadcast(true);
                socket.setSoTimeout(3000); // Chờ phản hồi trong 3 giây

                // 1. Gửi gói tin Broadcast qua tất cả interface có hỗ trợ broadcast
                byte[] sendData = REQUEST_MSG.getBytes();
                List<InetAddress> broadcastAddresses = getBroadcastAddresses();
                for (InetAddress broadcastAddress : broadcastAddresses) {
                    DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, broadcastAddress, PORT);
                    socket.send(sendPacket);
                    Log.d(TAG, "Đã gửi Broadcast tới: " + broadcastAddress.getHostAddress());
                }

                // 2. Chờ phản hồi từ Robot
                boolean found = false;
                byte[] receiveBuf = new byte[1024];
                while (isScanning) {
                    try {
                        DatagramPacket receivePacket = new DatagramPacket(receiveBuf, receiveBuf.length);
                        socket.receive(receivePacket);

                        String message = new String(receivePacket.getData(), 0, receivePacket.getLength());
                        Log.d(TAG, "Nhận phản hồi: " + message);

                        if (message.contains("ROBOT_HERE")) {
                            String robotIp = receivePacket.getAddress().getHostAddress();
                            if (callback != null) callback.onRobotFound(robotIp);
                            found = true;
                            break; // Tìm thấy thì dừng quét
                        }
                    } catch (SocketTimeoutException timeoutException) {
                        break;
                    }
                }

                if (!found && callback != null) {
                    callback.onError("Không tìm thấy robot trong mạng hiện tại");
                }
            } catch (Exception e) {
                Log.e(TAG, "Lỗi UDP: " + e.getMessage());
                if (callback != null) callback.onError(e.getMessage());
            } finally {
                if (socket != null) socket.close();
                isScanning = false;
            }
        }).start();
    }

    public void stopDiscovery() {
        isScanning = false;
    }

    private List<InetAddress> getBroadcastAddresses() throws Exception {
        Set<InetAddress> addresses = new LinkedHashSet<>();
        for (NetworkInterface networkInterface : Collections.list(NetworkInterface.getNetworkInterfaces())) {
            if (!networkInterface.isUp() || networkInterface.isLoopback()) {
                continue;
            }

            for (InterfaceAddress interfaceAddress : networkInterface.getInterfaceAddresses()) {
                InetAddress broadcast = interfaceAddress.getBroadcast();
                if (broadcast != null) {
                    addresses.add(broadcast);
                }
            }
        }

        if (addresses.isEmpty()) {
            addresses.add(InetAddress.getByName("255.255.255.255"));
        }

        return new ArrayList<>(addresses);
    }
}