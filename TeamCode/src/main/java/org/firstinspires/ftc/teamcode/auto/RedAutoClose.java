package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.intakePowerOffset;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.Constants.turretLookupTable;
import static org.firstinspires.ftc.teamcode.util.BallColor.EMPTY;
import static org.firstinspires.ftc.teamcode.util.ConversionUtil.convertPose2D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicSplineCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicStrafeCommand;
import org.firstinspires.ftc.teamcode.Commands.feeder.TurnFeederServoBoth;
import org.firstinspires.ftc.teamcode.Commands.intake.RunRightIntake;
import org.firstinspires.ftc.teamcode.Commands.intake.RunRightIntakeContinuous;
import org.firstinspires.ftc.teamcode.Commands.launcher.LaunchSequenceRight;
import org.firstinspires.ftc.teamcode.Commands.launcher.LaunchSequenceRightAuto;
import org.firstinspires.ftc.teamcode.Commands.launcher.LauncherSetDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.launcher.SetPowerLauncher;
import org.firstinspires.ftc.teamcode.Commands.launcher.SetRpmDistanceContinuous;
import org.firstinspires.ftc.teamcode.Commands.stopper.TurnStopperBoth;
import org.firstinspires.ftc.teamcode.Commands.stopper.TurnStopperLeft;
import org.firstinspires.ftc.teamcode.Commands.stopper.TurnStopperRight;
import org.firstinspires.ftc.teamcode.Commands.turret.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeSorterNoEnd;
import org.firstinspires.ftc.teamcode.Commands.launcher.SetFlywheelRpm;
import org.firstinspires.ftc.teamcode.Commands.launcher.SortedLuanchExtraSpin;
import org.firstinspires.ftc.teamcode.Commands.sorter.TurnToLaunchPattern;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretRotateContinuous;
import org.firstinspires.ftc.teamcode.Commands.vision.GetTagPattern;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.auto.paths.redAutoClose;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.StateIO;
import org.firstinspires.ftc.teamcode.state.TurretLookupTable;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.BallStoppers;
import org.firstinspires.ftc.teamcode.subsystems.FeederMotor;
import org.firstinspires.ftc.teamcode.subsystems.FeederServo;
import org.firstinspires.ftc.teamcode.subsystems.TurretRotate;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeLeft;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.TurretHood;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeRight;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

@Autonomous(name = "RedAutoClose", group = "ScrapHeads")
public class RedAutoClose extends CommandOpMode {

    // Subsystem
    private Drivetrain drivetrain;
    private Launcher launcher;
    private IntakeRight intakeRight;
    private TurretHood turretHood;
    private Vision vision;
    private FeederServo feederServo;
    private FeederMotor feederMotor;
    private TurretRotate turretRotate;
    private BallStoppers ballStoppers;

    public Boolean isBlue = false;

    public static final List<Pose2d> path = redAutoClose.PATH;

    public BallColor[] ballColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};

    @Override
    public void initialize() {
        // Init hardware + dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();
        turretLookupTable = new TurretLookupTable();

        setUpRobotState();

        // Subsystem
        drivetrain = new Drivetrain(hm, RobotState.getInstance().getEstimatedPose());
        drivetrain.register();

        launcher = new Launcher(hm);
        launcher.register();

        intakeRight = new IntakeRight(hm);
        intakeRight.register();

        feederMotor = new FeederMotor(hm);
        feederMotor.register();

        feederServo = new FeederServo(hm);
        feederServo.register();

        turretRotate = new TurretRotate(hm);
        turretRotate.register();

        turretRotate.resetEncoder();

        ballStoppers = new BallStoppers(hm);
        ballStoppers.register();

        turretHood = new TurretHood(hm);
        turretHood.register();

        vision = new Vision(hm);
        vision.register();

        // Custom constraints for some moves
        TurnConstraints turnConstraintsFast = new TurnConstraints(4, -4, 4);
        VelConstraint velConstraintFast = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(80),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintFast = new ProfileAccelConstraint(-40, 80);

        TurnConstraints turnConstraintsPickUp = new TurnConstraints(2, -2, 4);
        VelConstraint velConstraintPickUp = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(9),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintPickUp = new ProfileAccelConstraint(-10, 10);

        StateIO.save();

        TelemetryPacket p = new TelemetryPacket();
        Drawing.drawRobot(p.fieldOverlay(), convertPose2D(RobotState.getInstance().getEstimatedPose()));
        dashboard.sendTelemetryPacket(p);

        // Create the dive path the the robot follows in order
        SequentialCommandGroup followPath = new SequentialCommandGroup(
                new LauncherSetDefaultCommand(launcher),
                new SetPowerLauncher(launcher, 1),
                new InstantCommand(() -> turretRotate.setDefaultCommand(new TurretRotateContinuous(turretRotate))),

                new TurnStopperBoth(ballStoppers, BallStoppers.DOWN_ANGLE_LEFT, BallStoppers.DOWN_ANGLE_RIGHT),
                new TurnFeederServoBoth(feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE),

                new ParallelCommandGroup(
//                        new GetTagPattern(vision).withTimeout(5000),
//                        new SetFlywheelRpm(launcher, 3490),
//                        new SetHoodAngleCommand(turretHood, 1430),
                        new DynamicStrafeCommand(drivetrain, () -> path.get(1))
                ),

                new WaitCommand(1000),

                new SetFlywheelRpm(launcher, 4500),

//                new WaitUntilCommand(() -> launcher.isReadyToLaunch()),
                new LaunchSequenceRightAuto(feederServo, feederMotor, ballStoppers, intakeRight),

                new ParallelDeadlineGroup (
                        new DynamicStrafeCommand(drivetrain, () -> path.get(2)),
                        new RunRightIntakeContinuous(intakeRight, IntakeRight.INTAKE_POWER)
                ),

                new ParallelDeadlineGroup(
                        new DynamicStrafeCommand(drivetrain, () -> path.get(3)),
                        new RunRightIntakeContinuous(intakeRight, IntakeRight.INTAKE_POWER)
                ),

                new TurnStopperRight(ballStoppers, BallStoppers.DOWN_ANGLE_RIGHT),

                new ParallelDeadlineGroup(
                        new DynamicStrafeCommand(drivetrain, () -> path.get(4)),
                        new RunRightIntakeContinuous(intakeRight, IntakeRight.INTAKE_POWER)
                ),

                new LaunchSequenceRightAuto(feederServo, feederMotor, ballStoppers, intakeRight),

                new DynamicStrafeCommand(drivetrain, () -> path.get(5)),

                new ParallelDeadlineGroup(
                        new DynamicStrafeCommand(drivetrain, () -> path.get(6)),
                        new RunRightIntakeContinuous(intakeRight, IntakeRight.INTAKE_POWER)
                ),

//                new DynamicStrafeCommand(drivetrain, () -> path.get(7)),

                new SetFlywheelRpm(launcher, 4700),

                new TurnStopperRight(ballStoppers, BallStoppers.DOWN_ANGLE_RIGHT),

                new ParallelDeadlineGroup(
                        new DynamicStrafeCommand(drivetrain, () -> path.get(8)),
                        new RunRightIntakeContinuous(intakeRight, IntakeRight.INTAKE_POWER)
                ),

                new LaunchSequenceRightAuto(feederServo, feederMotor, ballStoppers, intakeRight),

                new ParallelDeadlineGroup(
                        new DynamicStrafeCommand(drivetrain, () -> path.get(9)),
                        new RunRightIntakeContinuous(intakeRight, IntakeRight.INTAKE_POWER)
                ),

                new ParallelDeadlineGroup(
                        new DynamicStrafeCommand(drivetrain, () -> path.get(10)),
                        new RunRightIntakeContinuous(intakeRight, IntakeRight.INTAKE_POWER)
                ),

                new SetFlywheelRpm(launcher, 4500),

                new TurnStopperRight(ballStoppers, BallStoppers.DOWN_ANGLE_RIGHT),

                new ParallelDeadlineGroup(
                        new DynamicStrafeCommand(drivetrain, () -> path.get(11),
                                turnConstraintsFast, velConstraintFast, accelConstraintFast),
                        new RunRightIntakeContinuous(intakeRight, IntakeRight.INTAKE_POWER)
                ),

                new LaunchSequenceRightAuto(feederServo, feederMotor, ballStoppers, intakeRight),

//                new DynamicStrafeCommand(drivetrain, () -> path.get(12)),

                new InstantCommand(StateIO::save)
        ) {
            // When the auto ends or gets interrupted will write to a jason file for auto -> tele data transfer.
            @Override
            public void end(boolean interrupted) {
                // Stop motors
                drivetrain.setDrivePowers(new ChassisSpeeds(0,0, 0));

                // Write the Auto -> teleop handoff
                StateIO.save();

                TelemetryPacket p = new TelemetryPacket();
                p.addLine("Saved the file in end");
                dashboard.sendTelemetryPacket(p);

                // telemetry/logging
                tele.addData("Auto ended", interrupted ? "interrupted" : "finished");
                tele.update();
            }
        };

        // Wait to start the auto path till the play button is pressed
        waitForStart();

        // Scheduled the sequential command group
        schedule(followPath);
    }

    public void setUpRobotState() {
        RobotState.getInstance().setAll(
                path.get(0),
//                new Pose2d(-2, -2, new Rotation2d(0.942478)),
                isBlue,
                ballColors,
                new ChassisSpeeds(0,0, 0)
        );

        RobotState.getInstance().setPattern(new BallColor[] {EMPTY, EMPTY, EMPTY});
    }

}
