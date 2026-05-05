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
    private final PrismAnimations.Solid color = new PrismAnimations.Solid(Color.RED);

    private final PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();

    private double currentPosX = 0;
    private double currentPosY = 0;
    private double pastPosX = 0;
    private double pastPosY = 0;
    public boolean isLaunching = false;

    public PrismLights(HardwareMap hm) {
        prism = hm.get(GoBildaPrismDriver.class,"prism");

        color.setPrimaryColor(Color.RED);
        color.setStartIndex(0);
        color.setStopIndex(24);

        rainbowSnakes.setNumberOfSnakes(2);
        rainbowSnakes.setSnakeLength(3);
        rainbowSnakes.setSpacingBetween(6);
        rainbowSnakes.setSpeed(0.5f);

        TelemetryPacket p = new TelemetryPacket();
        p.put("Device ID: ", prism.getDeviceID());
        dashboard.sendTelemetryPacket(p);
    }

    public void setColor (Color setColor) {
        color.setPrimaryColor(setColor);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, color);
    }
    public void setIsLaunching (Boolean isLaunching) {
        this.isLaunching = isLaunching;
    }

    public int i = 0;

    public boolean isInRange (double numOne, double numTwo, double range) {
        return Math.abs(Math.abs(numOne) - Math.abs(numTwo)) < range;
    }


    @Override
    public void periodic() {
        if (isLaunching) {
            setColor(Color.YELLOW);
            return;
        }
//        currentPosX = RobotState.getInstance().getEstimatedPose().getX();
//        currentPosY = RobotState.getInstance().getEstimatedPose().getY();
        if (!RobotState.getInstance().isMoving() &&
            RobotState.getInstance().isMoving()) {
            setColor(Color.GREEN);
        } else {
            setColor(Color.PINK);
        }

//        TelemetryPacket p = new TelemetryPacket();
//        p.put("Device ID: ", prism.getDeviceID());
//        p.put("VX", RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond);
//        p.put("VY", RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond);
//        p.put("Range x", Math.abs(Math.abs(currentPosX) - Math.abs(pastPosX)));
//        p.put("Range y", Math.abs(Math.abs(currentPosY) - Math.abs(pastPosY)));
//        dashboard.sendTelemetryPacket(p);

//        pastPosX = currentPosX;
//        pastPosY = currentPosY;

    }
}
