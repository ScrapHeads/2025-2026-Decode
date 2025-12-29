package org.firstinspires.ftc.teamcode.Commands.feeder;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FeederServo;

public class TurnFeederServoFront extends CommandBase {

    private final FeederServo feederServo;
    private final double frontAngle;

    public TurnFeederServoFront(FeederServo feederServo, double frontAngle) {
        this.feederServo = feederServo;
        this.frontAngle = frontAngle;
        addRequirements(feederServo);
    }

    @Override
    public void initialize() {
        feederServo.setFrontAngle(frontAngle);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }


}
