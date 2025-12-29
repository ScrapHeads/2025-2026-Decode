package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretRotate implements Subsystem {
    /**
     * Holds all tunable parameters and control state for the launcher.
     * Marked public so FTC Dashboard can reflect values in the Config tab.
     */
    public static class Params {
        public double currentAngle = 0.0;
    }

    public static LauncherHood.Params PARAMS = new LauncherHood.Params();

    public final ServoEx rotator;

    public TurretRotate (HardwareMap hm) {
        rotator = new SimpleServo(hm, "rotator", -1, 1);
        rotator.turnToAngle(0);
    }

    /**
     * Turns the servo a direction based on a -1 to 1 scale
     * @param power -1 to 1
     * */
    public void turnPower (double power) {
        rotator.turnToAngle(power);
    }

    public double getPower () {return rotator.getAngle();}


    @Override
    public void periodic() {
        tele.addData("Rotator power", "%.1f°", getPower());
    }


}
