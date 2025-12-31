package org.firstinspires.ftc.teamcode.Commands.feeder;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FeederMotor;

public class RunFeederMotor extends CommandBase {
    private final FeederMotor feederMotor;
    private final double power;

    public RunFeederMotor (FeederMotor feederMotor, double power) {
        this.feederMotor = feederMotor;
        this.power = power;

        addRequirements(feederMotor);
    }

    @Override
    public void initialize() {
        feederMotor.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
//        feederMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        // Continuous command — runs until canceled
        return true;
    }




}
