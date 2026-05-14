package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.prism.Color;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.state.RobotState;

public class PrismLightsAuto implements Subsystem {

    private final GoBildaPrismDriver prism;
    private final PrismAnimations.Solid color = new PrismAnimations.Solid();
    private final PrismAnimations.Solid color2 = new PrismAnimations.Solid();

    public PrismLightsAuto(HardwareMap hm, Boolean isFar) {
        prism = hm.get(GoBildaPrismDriver.class,"prism");

        color.setBrightness(50);
        color2.setBrightness(50);

        if (isFar) {
            color.setStartIndex(0);
            color.setStopIndex(6);

            color2.setStartIndex(18);
            color2.setStopIndex(24);
        } else {
            color.setStartIndex(6);
            color.setStopIndex(18);

            color2.setStartIndex(6);
            color2.setStopIndex(18);
        }

        if (RobotState.getInstance().isBlueTeam()) {
            setColor(Color.BLUE);
            setColor2(Color.BLUE);
        } else {
            setColor(Color.RED);
            setColor2(Color.RED);
        }

        TelemetryPacket p = new TelemetryPacket();
        p.put("Device ID: ", prism.getDeviceID());
        dashboard.sendTelemetryPacket(p);
    }

    public void setColor (Color setColor) {
        color.setPrimaryColor(setColor);
        color.setBrightness(50);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, color);
    }

    public void setColor2 (Color setColor) {
        color2.setPrimaryColor(setColor);
        color2.setBrightness(50);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, color2);
    }
}
