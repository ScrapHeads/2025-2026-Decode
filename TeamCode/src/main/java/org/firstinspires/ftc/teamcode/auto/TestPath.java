package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Rotation;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicStrafeCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicTurnCommand;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.auto.paths.testPath;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.StateIO;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TestPath", group = "ScrapHeads")
public class TestPath extends CommandOpMode {

    Drivetrain drivetrain;

    public boolean isBlue = true;

    public static final List<Pose2d> path = testPath.PATH;

    public BallColor[] ballColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};

    @Override
    public void initialize() {
        // Init hardware + dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        drivetrain = new Drivetrain(hm, path.get(0));
        drivetrain.register();

        // Custom constraints for some moves
        TurnConstraints turnConstraintsFast = new TurnConstraints(4, -4, 4);
        VelConstraint velConstraintFast = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(80),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintFast = new ProfileAccelConstraint(-40, 80);

        setUpRobotState();

        // Wait to start the auto path till the play button is pressed
        waitForStart();

        // Create the dive path the the robot follows in order
        SequentialCommandGroup followPath = new SequentialCommandGroup(
                new DynamicStrafeCommand(drivetrain, () -> new Pose2d(0, 20, new Rotation2d(Math.toRadians(180)))),
                new DynamicTurnCommand(drivetrain, () -> 0));
//                new DynamicStrafeCommand(drivetrain, () -> path.get(3)),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(4)));

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
    }
}
