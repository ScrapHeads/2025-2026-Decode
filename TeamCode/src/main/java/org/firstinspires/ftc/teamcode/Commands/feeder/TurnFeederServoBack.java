package org.firstinspires.ftc.teamcode.Commands.feeder;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FeederServo;

public class TurnFeederServoBack extends CommandBase {

    private final FeederServo feederServo;
    private final double backAngle;

    public TurnFeederServoBack(FeederServo feederServo, double frontAngle, double backAngle) {
        this.feederServo = feederServo;
        this.backAngle = backAngle;
        addRequirements(feederServo);
    }

    @Override
    public void initialize() {
        feederServo.setBackAngle(backAngle);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }


}
