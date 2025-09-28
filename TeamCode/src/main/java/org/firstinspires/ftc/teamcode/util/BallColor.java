package org.firstinspires.ftc.teamcode.util;

public enum BallColor {
    GREEN, PURPLE, EMPTY;

    public boolean isBall() {
        return this == GREEN || this == PURPLE;
    }
}