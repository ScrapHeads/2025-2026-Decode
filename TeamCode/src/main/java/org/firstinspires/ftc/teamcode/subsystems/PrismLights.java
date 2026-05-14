package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.prism.Color;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.state.RobotState;

public class PrismLights implements Subsystem {

    private final GoBildaPrismDriver prism;
    private final PrismAnimations.Solid color = new PrismAnimations.Solid();
    public boolean isLaunching = false;

    public PrismLights(HardwareMap hm) {
        prism = hm.get(GoBildaPrismDriver.class,"prism");

//        color.setPrimaryColor(Color.RED);
        color.setStartIndex(0);
        color.setStopIndex(24);

        color.setBrightness(50);

        TelemetryPacket p = new TelemetryPacket();
        p.put("Device ID: ", prism.getDeviceID());
        dashboard.sendTelemetryPacket(p);
    }

    public void setColor (Color setColor) {
        color.setPrimaryColor(setColor);
        color.setBrightness(50);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, color);
    }
    public void setIsLaunching (Boolean isLaunching) {
        this.isLaunching = isLaunching;
    }

    @Override
    public void periodic() {
        Color desiredColor;

        if (isLaunching) {
            desiredColor = Color.YELLOW;
        } else if (!RobotState.getInstance().isMoving()
                && !RobotState.getInstance().isTurretRotating()
                && RobotState.getInstance().isTurretInRange()) {
            desiredColor = Color.GREEN;
        } else {
            desiredColor = Color.WHITE;
        }

        if (color.getPrimaryColor() != desiredColor) {
            setColor(desiredColor);
        }

        TelemetryPacket p = new TelemetryPacket();
        p.put("Prism color", color.getPrimaryColor());
        p.put("Color brightness", color.getBrightness());
        p.put("Is Launching", isLaunching);
        p.put("Is Moving", RobotState.getInstance().isMoving());
        p.put("Is Turret Rotating", RobotState.getInstance().isTurretRotating());
        dashboard.sendTelemetryPacket(p);
    }
}
