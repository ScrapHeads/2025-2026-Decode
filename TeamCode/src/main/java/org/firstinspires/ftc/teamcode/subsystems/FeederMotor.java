package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.intake.BallTracker;

public class FeederMotor implements Subsystem {


    public static final double UP_POWER = -1;
    public static final double DOWN_POWER = 1;

    private final MotorEx feeder;
    private BallTracker ballTracker = null;


    public FeederMotor(HardwareMap hm) {
        feeder = new MotorEx(hm, "feeder");
    }

    public FeederMotor(HardwareMap hm, BallTracker ballTracker) {
        feeder = new MotorEx(hm, "feeder");
        this.ballTracker = ballTracker;
    }

    public void setPower (double power) {
        feeder.set(power);

        if (ballTracker != null){
            ballTracker.setFeederPower(-power);
        } else {
            tele.addLine("Ball Tracker null feeder");
        }
    }

    public void stopMotor () {
        feeder.stopMotor();

        if (ballTracker != null) {
            ballTracker.setFeederPower(0);
        } else {
            tele.addLine("Ball Tracker null feeder");
        }
    }

}
