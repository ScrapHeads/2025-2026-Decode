package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.feeder.RunFeederMotor;
import org.firstinspires.ftc.teamcode.Commands.feeder.TurnFeederServoBoth;
import org.firstinspires.ftc.teamcode.Commands.intake.RunLeftIntake;
import org.firstinspires.ftc.teamcode.Commands.stopper.TurnStopperBoth;
import org.firstinspires.ftc.teamcode.Commands.stopper.TurnStopperLeft;
import org.firstinspires.ftc.teamcode.subsystems.intake.BallStoppers;
import org.firstinspires.ftc.teamcode.subsystems.FeederMotor;
import org.firstinspires.ftc.teamcode.subsystems.FeederServo;
import org.firstinspires.ftc.teamcode.subsystems.turret.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretRotate;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeLeft;

public class LaunchSequenceLeft extends SequentialCommandGroup {

    public LaunchSequenceLeft(FeederServo feederServo, FeederMotor feederMotor, BallStoppers ballStoppers,
                              IntakeLeft intakeLeft, TurretRotate turretRotate, Launcher launcher) {
        addCommands(
                new WaitUntilCommand(() -> turretRotate.isInRange()),
//                new WaitUntilCommand(() -> launcher.isReadyToLaunch()),
                new RunFeederMotor(feederMotor, FeederMotor.UP_POWER),
                new WaitCommand(100),
                new TurnStopperBoth(ballStoppers, BallStoppers.DOWN_ANGLE_LEFT, BallStoppers.DOWN_ANGLE_RIGHT),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new RunLeftIntake(intakeLeft, .8),
//                new WaitCommand(800),
                new WaitCommand(25),
                new TurnStopperLeft(ballStoppers, BallStoppers.UP_ANGLE_LEFT),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
                new WaitCommand(300),
                new TurnStopperLeft(ballStoppers, BallStoppers.DOWN_ANGLE_LEFT),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new WaitCommand(25),
                new TurnStopperLeft(ballStoppers, BallStoppers.UP_ANGLE_LEFT),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
                new WaitCommand(300),
                new TurnStopperLeft(ballStoppers, BallStoppers.DOWN_ANGLE_LEFT),
                new TurnFeederServoBoth(feederServo, FeederServo.OUT_FRONT_ANGLE, FeederServo.OUT_BACK_ANGLE),
                new WaitCommand(25),
                new TurnStopperLeft(ballStoppers, BallStoppers.UP_ANGLE_LEFT),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),
                new WaitCommand(300),
                new RunLeftIntake(intakeLeft, 0),
                new RunFeederMotor(feederMotor, 0)
        );
    }

}
