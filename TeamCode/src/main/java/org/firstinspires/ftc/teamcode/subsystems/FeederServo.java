package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FeederServo implements Subsystem {

    private final ServoEx frontFeeder;
    private final ServoEx backFeeder;

    // Physical range of the servo
    public static final double MIN_ANGLE_SERVO = 500;
    public static final double MAX_ANGLE_SERVO = 2500;

    public static final double IN_FRONT_ANGLE = 1811;
    public static final double OUT_FRONT_ANGLE = 1535;

    public static final double IN_BACK_ANGLE = 1313;
    public static final double OUT_BACK_ANGLE = 1548;

    public FeederServo (HardwareMap hm) {
        frontFeeder = new SimpleServo(hm, "feederFront", MIN_ANGLE_SERVO, MAX_ANGLE_SERVO);
        backFeeder = new SimpleServo(hm, "feederBack", MIN_ANGLE_SERVO, MAX_ANGLE_SERVO);

    }

    public void setBothAngle (double frontAngle, double backAngle) {
        setFrontAngle(frontAngle);
        setBackAngle(backAngle);
    }
    public void setFrontAngle(double frontAngle) {
//        double safeAngle = Math.min(OUT_FRONT_ANGLE, Math.max(IN_FRONT_ANGLE, frontAngle));
        frontFeeder.turnToAngle(frontAngle);
    }
    public void setBackAngle(double backAngle) {
//        double safeAngle = Math.min(IN_BACK_ANGLE, Math.max(OUT_BACK_ANGLE, backAngle));
        backFeeder.turnToAngle(backAngle);
    }


}
