package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FeederMotor implements Subsystem {

    public static final double UP_POWER = -1;
    public static final double DOWN_POWER = 1;

    private final MotorEx feeder;

    public FeederMotor(HardwareMap hm) {
        feeder = new MotorEx(hm, "feeder");
    }

    public void setPower (double power) {
        feeder.set(power);
    }

    public void stopMotor () {
        feeder.stopMotor();
    }


}
