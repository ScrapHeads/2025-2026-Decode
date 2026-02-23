package org.firstinspires.ftc.teamcode.subsystems.intake;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.util.BallColor.GREEN;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.BallColor;

/**
 * The IntakeSubsystem controls the intake motor responsible for
 * collecting and feeding game elements.
 *
 * <p>Provides basic methods for setting power and stopping.
 */
public class IntakeLeft implements Subsystem  {

    private final MotorEx intakeMotorLeft;

//    private final RevColorSensorV3 colorSensorLeft;

    // Default power constants (adjust as needed)
    public static final double INTAKE_POWER = 1.0;
    public static final double OUTTAKE_POWER = -0.4;
    public static final double REST_POWER = 0.0;

    public static final double MOTOR_TPR   = 28;   // ticks per motor rev
    public static final double GEAR_RATIO  = 1;  // motor:wheel upgear
    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;

    /**
     * Constructs the IntakeSubsystem.
     *
     * @param hm The HardwareMap used to retrieve the motor device.
     */
    public IntakeLeft(HardwareMap hm) {
        intakeMotorLeft = new MotorEx(hm, "intakeLeft"); // name must match configuration

        intakeMotorLeft.setInverted(false);

        intakeMotorLeft.setZeroPowerBehavior(BRAKE);

//        colorSensorLeft = hm.get(RevColorSensorV3.class, "colorLeft");
//
//        colorSensorLeft.setGain(10);
//
//        colorSensorLeft.enableLed(true);

        // ensure stopped at init
        stopIntake();
    }

    public void setLeftPower (double power) {
        intakeMotorLeft.set(power);
    }

    /**
     * Stops the both intake motor immediately.
     */
    public void stopIntake() {
        intakeMotorLeft.stopMotor();
    }

    public void stopLeft () {intakeMotorLeft.stopMotor();}

    /**
     * @return The current power being applied to the intake motor.
     */
    public double getPowerLeft() {
        return intakeMotorLeft.get();
    }

    public BallColor detectBallColor (RevColorSensorV3 colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        // --- Normalize readings to minimize lighting variance ---
        double total = r + g + b;
        if (total == 0) return BallColor.EMPTY;

        double rNorm = r / total;
        double gNorm = g / total;
        double bNorm = b / total;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Color Sensor Right", String.format("%.2f, %.2f, %.2f",
                rNorm, gNorm, bNorm));
        dashboard.sendTelemetryPacket(packet);

        // --- GREEN detection: strong green dominance ---
        if (gNorm > rNorm * 2.8 && gNorm > bNorm * 1.2) {
            return GREEN;
        }

        // --- PURPLE detection: red + blue high, green low ---
        double avgRB = (rNorm + bNorm) / 1.87;
        if (avgRB > gNorm) {
            return BallColor.PURPLE;
        }

        // --- None detected ---
        return BallColor.EMPTY;
    }

    @Override
    public void periodic() {
        tele.addData("Intake Power", toString());

        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Color Left", detectBallColor(colorSensorLeft));
//        packet.put("Color Sensor Left", String.format("%d, %d, %d",
//                colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue()));
        dashboard.sendTelemetryPacket(packet);
    }
}
