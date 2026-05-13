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

    public PrismLightsAuto(HardwareMap hm) {
        prism = hm.get(GoBildaPrismDriver.class,"prism");

//        color.setPrimaryColor(Color.RED);
        color.setStartIndex(0);
        color.setStopIndex(24);

        color.setBrightness(50);

        if (RobotState.getInstance().getTeam()) {
            setColor(Color.BLUE);
        } else {
            setColor(Color.RED);
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
}
