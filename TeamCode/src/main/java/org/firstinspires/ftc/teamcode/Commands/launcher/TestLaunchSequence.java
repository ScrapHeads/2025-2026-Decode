package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.feeder.RunFeederMotor;
import org.firstinspires.ftc.teamcode.Commands.feeder.TurnFeederServoBoth;
import org.firstinspires.ftc.teamcode.Commands.intake.RunRightIntake;
import org.firstinspires.ftc.teamcode.subsystems.FeederMotor;
import org.firstinspires.ftc.teamcode.subsystems.FeederServo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class TestLaunchSequence extends SequentialCommandGroup {

    public TestLaunchSequence (FeederServo feederServo, FeederMotor feederMotor, Intake intake) {
        addCommands(
                new RunFeederMotor(feederMotor, -1),
                new WaitCommand(100),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new WaitCommand(75),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
                new RunRightIntake(intake, Intake.INTAKE_POWER),
                new WaitCommand(300),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new WaitCommand(75),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
                new WaitCommand(300),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new WaitCommand(75),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
                new WaitCommand(300),
                new RunRightIntake(intake, 0),
                new RunFeederMotor(feederMotor, 0)
        );
    }

}
