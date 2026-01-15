package org.firstinspires.ftc.teamcode.Commands.feeder;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FeederServo;

public class TurnFeederServoBothContinuous extends CommandBase {

    private final FeederServo feederServo;
    private final double frontAngle;
    private final double backAngle;

    public TurnFeederServoBothContinuous(FeederServo feederServo, double frontAngle, double backAngle) {
        this.feederServo = feederServo;
        this.frontAngle = frontAngle;
        this.backAngle = backAngle;
        addRequirements(feederServo);
    }

    @Override
    public void execute () {
        feederServo.setBothAngle(frontAngle, backAngle);
    }
}
