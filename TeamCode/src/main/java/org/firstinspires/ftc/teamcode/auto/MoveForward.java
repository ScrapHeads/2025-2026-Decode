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
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.TurnConstraints;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicStrafeCommand;
//import org.firstinspires.ftc.teamcode.auto.paths.MoveForwardPath;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//
//import java.util.Arrays;
//import java.util.List;
//
//@Autonomous(name = "MoveForward", group = "ScrapHeads")
//public class MoveForward extends CommandOpMode {
//
//    Drivetrain drivetrain;
//
//    public static final List<Pose2d> testPath = MoveForwardPath.PATH;
//
//    @Override
//    public void initialize() {
//        // Init hardware + dashboard
//        hm = hardwareMap;
//        tele = telemetry;
//        dashboard = FtcDashboard.getInstance();
//
//        drivetrain = new Drivetrain(hm, testPath.get(0));
//        drivetrain.register();
//
//        // Custom constraints for some moves
//        TurnConstraints turnConstraintsFast = new TurnConstraints(4, -4, 4);
//        VelConstraint velConstraintFast = new MinVelConstraint(Arrays.asList(
//                drivetrain.kinematics.new WheelVelConstraint(80),
//                new AngularVelConstraint(Math.PI)));
//        AccelConstraint accelConstraintFast = new ProfileAccelConstraint(-40, 80);
//
//        waitForStart();
//
//        // Run a sequence of movements dynamically using live pose grabbing
//        schedule(new SequentialCommandGroup(
//                // Move to position 1 using defaults
//                new DynamicStrafeCommand(drivetrain, () -> testPath.get(1))
//        ));
//    }
//}
