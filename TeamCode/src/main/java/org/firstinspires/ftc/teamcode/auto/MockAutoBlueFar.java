//package org.firstinspires.ftc.teamcode.auto;
//
//import static org.firstinspires.ftc.teamcode.Constants.dashboard;
//import static org.firstinspires.ftc.teamcode.Constants.hm;
//import static org.firstinspires.ftc.teamcode.Constants.tele;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.TurnConstraints;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicStrafeCommand;
//import org.firstinspires.ftc.teamcode.auto.paths.mockAutoBlueFar;
//import org.firstinspires.ftc.teamcode.state.RobotState;
//import org.firstinspires.ftc.teamcode.state.StateIO;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//
//import java.util.Arrays;
//import java.util.List;
//
//@Autonomous(name = "MockAutoBlueFar", group = "ScrapHeads")
//public class MockAutoBlueFar extends CommandOpMode {
//
//    Drivetrain drivetrain;
//
//    public static final List<Pose2d> path = mockAutoBlueFar.PATH;
//
//    @Override
//    public void initialize() {
//        // Init hardware + dashboard
//        hm = hardwareMap;
//        tele = telemetry;
//        dashboard = FtcDashboard.getInstance();
//
//        drivetrain = new Drivetrain(hm, path.get(0));
//        drivetrain.register();
//
//        // Custom constraints for some moves
//        TurnConstraints turnConstraintsFast = new TurnConstraints(4, -4, 4);
//        VelConstraint velConstraintFast = new MinVelConstraint(Arrays.asList(
//                drivetrain.kinematics.new WheelVelConstraint(80),
//                new AngularVelConstraint(Math.PI)));
//        AccelConstraint accelConstraintFast = new ProfileAccelConstraint(-40, 80);
//
//        // Wait to start the auto path till the play button is pressed
//        waitForStart();
//
//        // Create the dive path the the robot follows in order
//        SequentialCommandGroup followPath = new SequentialCommandGroup(
//                new DynamicStrafeCommand(drivetrain, () -> path.get(1)),
//                new WaitCommand(3000),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(2)),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(3)),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(4)),
//                new WaitCommand(3000),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(5)),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(6)),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(7)),
//                new WaitCommand(3000),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(8)),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(9)),
//                new DynamicStrafeCommand(drivetrain, () -> path.get(10)),
//                new WaitCommand(3000)
//        ) {
//            // When the auto ends or gets interrupted will write to a jason file for auto -> tele data transfer.
//            @Override
//            public void end(boolean interrupted) {
//                // Stop motors
//                drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0) , 0));
//
//                // Write the Auto -> teleop handoff
//                writeAutoHandoff();
//
//                // telemetry/logging
//                tele.addData("Auto ended", interrupted ? "interrupted" : "finished");
//                tele.update();
//            }
//        };
//
//        // Scheduled the sequential command group
//        schedule(followPath);
//    }
//
//    private void writeAutoHandoff() {
//        try {
//            // Update the pose of the robot
//            drivetrain.updatePoseEstimate();
//
//            // Get the updated robot pose
//            Pose2d pose = drivetrain.localizer.getPose();
//            boolean isBlue = true;
//
//            // Create a new RobotState class
//            RobotState rs = new RobotState(pose, isBlue);
//
//            // Save the RobotState class to the json file
//            StateIO.save(rs);
//
//        } catch (Exception e) {
//            // Keep Auto safe-avoid throwing out of end(); add a log
//            tele.addData("Handoff write error", e.getMessage());
//            tele.update();
//        }
//    }
//}
