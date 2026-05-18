package org.firstinspires.ftc.teamcode.Commands.launcher.launchsequence;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.feeder.RunFeederMotor;
import org.firstinspires.ftc.teamcode.Commands.feeder.TurnFeederServoBoth;
import org.firstinspires.ftc.teamcode.Commands.intake.RunRightIntake;
import org.firstinspires.ftc.teamcode.Commands.stopper.TurnStopperBoth;
import org.firstinspires.ftc.teamcode.Commands.stopper.TurnStopperRight;
import org.firstinspires.ftc.teamcode.subsystems.intake.BallStoppers;
import org.firstinspires.ftc.teamcode.subsystems.FeederMotor;
import org.firstinspires.ftc.teamcode.subsystems.FeederServo;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeRight;

public class LaunchSequenceRightAuto extends SequentialCommandGroup {

    public LaunchSequenceRightAuto(FeederServo feederServo, FeederMotor feederMotor, BallStoppers ballStoppers,
                                   IntakeRight intakeRight) {
        addCommands(
                new RunFeederMotor(feederMotor, FeederMotor.UP_POWER),
                new WaitCommand(75),
                new TurnStopperBoth(ballStoppers, BallStoppers.DOWN_ANGLE_LEFT, BallStoppers.DOWN_ANGLE_RIGHT),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new RunRightIntake(intakeRight, .8),
//                new WaitCommand(800),
                new WaitCommand(25),
                new TurnStopperRight(ballStoppers, BallStoppers.UP_ANGLE_RIGHT),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
                new WaitCommand(285),
                new TurnStopperRight(ballStoppers, BallStoppers.DOWN_ANGLE_RIGHT),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new WaitCommand(25),
                new TurnStopperRight(ballStoppers, BallStoppers.UP_ANGLE_RIGHT),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
                new WaitCommand(285),
                new TurnStopperRight(ballStoppers, BallStoppers.DOWN_ANGLE_RIGHT),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new WaitCommand(25),
                new TurnStopperRight(ballStoppers, BallStoppers.UP_ANGLE_RIGHT),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
//                new WaitCommand(300),
                new ParallelCommandGroup(
                        new RunRightIntake(intakeRight, 0),
                        new RunFeederMotor(feederMotor, 0)
                )
        );
    }

}
