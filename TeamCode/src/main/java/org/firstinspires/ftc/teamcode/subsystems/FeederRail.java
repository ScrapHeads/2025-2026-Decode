package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The FeederRail subsystem controls the servo mechanism that
 * drops or retracts the rail used for feeding balls into the launcher.
 *
 * <p>This uses the FTCLib ServoEx class for smoother and safer control.
 */
public class FeederRail implements Subsystem {

    private final ServoEx feederServo;
    private boolean deployed = false;

    // === Configurable positions ===
    public static final double DEPLOY_POS = 210;   // adjust as needed
    public static final double RETRACT_POS = 30;  // adjust as needed

    /**
     * Initializes the feeder rail using the given hardware map.
     *
     * @param hm the HardwareMap to retrieve the servo device
     */
    public FeederRail(HardwareMap hm) {
        feederServo = new SimpleServo(hm, "feederRail", 0, 270);

        feederServo.setRange(0.0, 1.0);   // logical safety clamp
        // Optional: set servo direction, limits, or default speed
        feederServo.turnToAngle(RETRACT_POS);
        deployed = false;
    }

    /** Deploys the feeder rail (down position to feed ball). */
    public void deploy() {
        feederServo.turnToAngle(DEPLOY_POS);
        deployed = true;
    }

    /** Retracts the feeder rail (up position to clear the launcher). */
    public void retract() {
        feederServo.turnToAngle(RETRACT_POS);
        deployed = false;
    }

    /** Toggles between deployed and retracted states. */
    public void toggle() {
        if (deployed) retract();
        else deploy();
    }

    /** @return true if the feeder rail is deployed */
    public boolean isDeployed() {
        return deployed;
    }

    @Override
    public void periodic() {
        tele.addData("Feeder Rail", deployed ? "Deployed" : "Retracted");
        tele.addData("Feeder Servo Pos", "%.2f", feederServo.getPosition());
    }
}
