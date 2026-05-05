package com.n7.quanlyrobotquetnha.models;

public class MoveInfo {
    public String type;
    public float value;

    private MoveInfo(String go, float value) {
        this.type = go;
        this.value = value;
    }

    public static MoveInfo go(float value) {
        return new MoveInfo("go", value);
    }

    public static MoveInfo rotate(float value) {
        return new MoveInfo("rotate", value);
    }
}
