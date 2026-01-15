package org.firstinspires.ftc.teamcode.Commands.feeder;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FeederMotor;

public class RunFeederMotorContinuous extends CommandBase {
    private final FeederMotor feederMotor;
    private final double power;

    public RunFeederMotorContinuous(FeederMotor feederMotor, double power) {
        this.feederMotor = feederMotor;
        this.power = power;

        addRequirements(feederMotor);
    }

    @Override
    public void execute () {
        feederMotor.setPower(power);
    }


    @Override
    public void end(boolean interrupted) {
//        feederMotor.stopMotor();
    }
}
