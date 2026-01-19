package org.firstinspires.ftc.teamcode.Commands.launcher;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class SetRpmDistanceContinuous extends CommandBase {

    private final Launcher launcher;
    double count = 0;

    public SetRpmDistanceContinuous (Launcher launcher) {
        this.launcher = launcher;

        addRequirements(launcher);
    }

    @Override
    public void initialize () {
        launcher.enable();
    }

    @Override
    public void execute() {
        TelemetryPacket p = new TelemetryPacket();
        p.put("A Call count setRpm", count++);
        dashboard.sendTelemetryPacket(p);
        launcher.getAndSetFlywheelByDistance();
    }

    @Override
    public void end (boolean interrupted) {
//        launcher.disable();
    }
}
