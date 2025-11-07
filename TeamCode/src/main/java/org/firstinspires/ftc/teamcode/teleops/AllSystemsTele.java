package org.firstinspires.ftc.teamcode.teleops;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeSorter;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeSorterNoEnd;
import org.firstinspires.ftc.teamcode.Commands.intake.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.launcher.SetFlywheelRpm;
import org.firstinspires.ftc.teamcode.Commands.launcher.ShootAllLoaded;
import org.firstinspires.ftc.teamcode.Commands.launcher.SortedLuanch;
import org.firstinspires.ftc.teamcode.Commands.launcher.StopFlywheel;
import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.Commands.vision.GetTagPattern;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
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

@TeleOp(name = "AllSystemsTele", group = "ScrapHeads")
public class AllSystemsTele extends CommandOpMode {
    // Controller
    private GamepadEx driver;

    // Subsystem
    private Drivetrain drivetrain;
    private LauncherBall launcher;
    private Sorter sorter;
//    private FeederRail feederRail;
    private HoldControl holdControl;
    private Intake intake;
    private LauncherHood hood;
    private Vision vision;

    @Override
    public void initialize() {
        // Initialize shared constants
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        StateIO.load();

        // Initialize the subsystems declared at the top of the code
        drivetrain = new Drivetrain(hm, RobotState.getInstance().getOdometryPose());
        drivetrain.register();

        //TODO Comment out WHEN DOING Matches
//        RobotState.getInstance().setPattern(new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN});

        // Gamepad
        driver = new GamepadEx(gamepad1);

        // Subsystem
        launcher = new LauncherBall(hm);
        launcher.register();

        sorter = new Sorter(hm);
        sorter.register();

//        feederRail = new FeederRail(hm);
//        feederRail.register();

        holdControl = new HoldControl(hm);
        holdControl.register();

        intake = new Intake(hm);
        intake.register();

        hood = new LauncherHood(hm);
        hood.register();

        vision = new Vision(hm);
        vision.register();

        // Bind controls
        assignControls();

        tele.addData("What index for sorter: ", sorter.getCurrentIndex());
        tele.addData("Color for sorter: ", Arrays.toString(RobotState.getInstance().getBallColors()));
        tele.addData("Team isBlue", RobotState.getInstance().getTeam());
        tele.addData("Heading offset", RobotState.getInstance().getHeadingOffset());
        tele.update();
    }

    private void assignControls() {
        // Set up continuous drive
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

        // A: spin up and hold 6000 RPM (command ends only when launcher.disable() is called)
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1)
                .whenActive(new IntakeSorterNoEnd(intake, sorter, holdControl, Intake.INTAKE_POWER))
                .whenInactive(new RunIntakeCommand(intake, 0));
//                .whenActive(new SetPowerLauncher(launcher, 1));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1)
                .whileActiveOnce(new RunIntakeCommand(intake, Intake.OUTTAKE_POWER))
                .whenInactive(new RunIntakeCommand(intake, 0));

        // Left Trigger:
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1)
//                .whenActive(new StopFlywheel(launcher));

        driver.getGamepadButton(RIGHT_BUMPER)
                .whenPressed(new TurnOneSlot(sorter, Sorter.CCW_DIRECTION));

        driver.getGamepadButton(LEFT_BUMPER)
                .whenPressed(new TurnOneSlot(sorter, Sorter.CW_DIRECTION));

        driver.getGamepadButton(A)
                .whenPressed(new ShootAllLoaded(launcher, sorter, holdControl));

        driver.getGamepadButton(B)
                .whenPressed(new SortedLuanch(launcher, sorter, holdControl));

        driver.getGamepadButton(X)
                        .whenPressed(new GetTagPattern(vision));

//        driver.getGamepadButton(A)
//                        .whenPressed(new HoldControlCommand(holdControl, HoldControl.HoldPosition.LAUNCHING));
//
//        driver.getGamepadButton(B)
//                .whenPressed(new HoldControlCommand(holdControl, HoldControl.HoldPosition.TRANSPORT));

        driver.getGamepadButton(DPAD_UP)
                .whenPressed(new ParallelCommandGroup(
                        new SetFlywheelRpm(launcher, 4450),
                        new SetHoodAngleCommand(hood, LauncherHood.HIGH_SHOOT_ANGLE)
                ))
                        ;

        driver.getGamepadButton(DPAD_LEFT)
                        .whenPressed(new ParallelCommandGroup(
                                new SetFlywheelRpm(launcher, 3000),
                                new SetHoodAngleCommand(hood, LauncherHood.LOW_SHOOT_ANGLE)
                        ));

        driver.getGamepadButton(DPAD_DOWN)
                .whenPressed(new StopFlywheel(launcher));

//        driver.getGamepadButton(DPAD_UP)
//                .whenPressed(new SetHoodAngleCommand(hood, 0));
    }
}

