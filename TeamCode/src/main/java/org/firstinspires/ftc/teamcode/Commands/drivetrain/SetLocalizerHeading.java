package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class SetLocalizerHeading extends CommandBase {

    private final Drivetrain drivetrain;
    private final double heading;


    public SetLocalizerHeading (Drivetrain drivetrain, double heading) {
        this.drivetrain = drivetrain;
        this.heading = heading;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        TelemetryPacket p = new TelemetryPacket();
        p.addLine("Calling set heading");
        drivetrain.localizer.setHeading(heading);
    }

}
