package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.util.BallColor.EMPTY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicStrafeCommand;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeSorter;
import org.firstinspires.ftc.teamcode.Commands.launcher.SetFlywheelRpm;
import org.firstinspires.ftc.teamcode.Commands.launcher.SortedLuanch;
import org.firstinspires.ftc.teamcode.Commands.vision.GetTagPattern;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.auto.paths.blueAutoClose;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.StateIO;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;
import org.firstinspires.ftc.teamcode.subsystems.LauncherHood;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.Arrays;
import java.util.List;

public class BlueAutoClose extends CommandOpMode {

    private Drivetrain drivetrain;

    private LauncherBall launcher;
    private LauncherHood hood;
    private Intake intake;
    private HoldControl holdControl;
    private Sorter sorter;
    private Vision vision;

    public boolean isBlue = true;

    public static final List<Pose2d> path = blueAutoClose.PATH;

    public BallColor[] ballColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};

    @Override
    public void initialize() {
        // Init hardware + dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        setUpRobotState();

        drivetrain = new Drivetrain(hm, path.get(0));
        drivetrain.register();

        launcher = new LauncherBall(hm);
        launcher.register();

        hood = new LauncherHood(hm);
        hood.register();

        intake = new Intake(hm);
        intake.register();

        holdControl = new HoldControl(hm);
        holdControl.register();

        sorter = new Sorter(hm);
        sorter.register();

        vision = new Vision(hm, drivetrain);
        vision.register();

        // Custom constraints for some moves
        TurnConstraints turnConstraintsFast = new TurnConstraints(4, -4, 4);
        VelConstraint velConstraintFast = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(80),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintFast = new ProfileAccelConstraint(-40, 80);

        // Wait to start the auto path till the play button is pressed
        waitForStart();

        // Create the dive path the the robot follows in order
        SequentialCommandGroup followPath = new SequentialCommandGroup(
                new GetTagPattern(vision).raceWith(
                        new DynamicStrafeCommand(drivetrain, () -> path.get(1))
                ),
                new SetFlywheelRpm(launcher, 4400),
                new DynamicStrafeCommand(drivetrain, () -> path.get(2)),
                new SortedLuanch(launcher, sorter, holdControl, 300),
                new IntakeSorter(intake, sorter, holdControl, Intake.INTAKE_POWER).raceWith(
                        new SequentialCommandGroup(
                                new DynamicStrafeCommand(drivetrain, () -> path.get(3)),
                                new DynamicStrafeCommand(drivetrain, () -> path.get(4))
                        )
                ),
                new DynamicStrafeCommand(drivetrain, () -> path.get(5)),
                new SortedLuanch(launcher, sorter, holdControl, 300),
                new IntakeSorter(intake, sorter, holdControl, Intake.INTAKE_POWER).raceWith(
                        new SequentialCommandGroup(
                                new DynamicStrafeCommand(drivetrain, () -> path.get(6)),
                                new DynamicStrafeCommand(drivetrain, () -> path.get(7))
                        )
                ),
                new DynamicStrafeCommand(drivetrain, () -> path.get(8)),
                new SortedLuanch(launcher, sorter, holdControl, 300),
                new DynamicStrafeCommand(drivetrain, () -> path.get(9))
//                new DynamicStrafeCommand(drivetrain, () -> path.get(10))
//                new DynamicStrafeCommand(drivetrain, () -> path.get(11))
//                new SortedLuanch(launcher, sorter, holdControl, 300),
        ) {
            // When the auto ends or gets interrupted will write to a jason file for auto -> tele data transfer.
            @Override
            public void end(boolean interrupted) {
                // Stop motors
                drivetrain.setDrivePowers(new ChassisSpeeds(0,0, 0));

                // Write the Auto -> teleop handoff
                StateIO.save();

                // telemetry/logging
                tele.addData("Auto ended", interrupted ? "interrupted" : "finished");
                tele.update();
            }
        };

        // Scheduled the sequential command group
        schedule(followPath);
    }

    public void setUpRobotState() {
        RobotState.getInstance().setAll(
                path.get(0),
                isBlue,
                ballColors,
                new ChassisSpeeds(0,0, 0)
        );

        RobotState.getInstance().setPattern(new BallColor[] {EMPTY, EMPTY, EMPTY});
    }

}
