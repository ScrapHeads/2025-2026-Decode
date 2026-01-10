package org.firstinspires.ftc.teamcode.teleops;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drivetrain.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.intake.RunBothIntakesContinuous;
import org.firstinspires.ftc.teamcode.Commands.intake.RunLeftIntake;
import org.firstinspires.ftc.teamcode.Commands.intake.RunLeftIntakeContinuous;
import org.firstinspires.ftc.teamcode.Commands.intake.RunRightIntake;
import org.firstinspires.ftc.teamcode.Commands.intake.RunRightIntakeContinuous;
import org.firstinspires.ftc.teamcode.Commands.launcher.SetPowerLauncher;
import org.firstinspires.ftc.teamcode.Commands.launcher.StopFlywheel;
import org.firstinspires.ftc.teamcode.Commands.launcher.TestLaunchSequence;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.StateIO;
import org.firstinspires.ftc.teamcode.subsystems.BallStoppers;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FeederMotor;
import org.firstinspires.ftc.teamcode.subsystems.FeederServo;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeLeft;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.TurretRotate;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeRight;

@TeleOp(name = "AllSystemsTeleRed", group = "ScrapHeads")
public class AllSystemsTeleRed extends CommandOpMode {
    // Controller
    private GamepadEx driver;

    // Subsystem
    private Drivetrain drivetrain;
    private Launcher launcher;
    private IntakeLeft intakeLeft;
    private IntakeRight intakeRight;
//    private LauncherHood hood;
//    private Vision vision;
    private FeederServo feederServo;
    private FeederMotor feederMotor;
    private TurretRotate turretRotate;
    private BallStoppers ballStoppers;

    private Pose2d setLaunchPoint;

    @Override
    public void initialize() {
        // Initialize shared constants
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        StateIO.load();

        //TODO Comment out WHEN DOING Matches
//        RobotState.getInstance().setPattern(new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN});
//        RobotState.getInstance().setTeam(true);
        RobotState.getInstance().setTeam(false);

        // Initialize the subsystems declared at the top of the code
        drivetrain = new Drivetrain(hm, RobotState.getInstance().getOdometryPose());
        drivetrain.register();

        // Gamepad
        driver = new GamepadEx(gamepad1);

        // Subsystem
        launcher = new Launcher(hm);
        launcher.register();

        intakeLeft = new IntakeLeft(hm);
        intakeLeft.register();

        intakeRight = new IntakeRight(hm);
        intakeRight.register();

        feederMotor = new FeederMotor(hm);
        feederMotor.register();

        feederServo = new FeederServo(hm);
        feederServo.register();

        turretRotate = new TurretRotate(hm);
        turretRotate.register();

        ballStoppers = new BallStoppers(hm);
        ballStoppers.register();

//        hood = new LauncherHood(hm);
//        hood.register();

//        vision = new Vision(hm);
//        vision.register();

        // Bind controls
        assignControls();

        CommandScheduler.getInstance().cancelAll();

//        vision.setPipeline(0);

        if (!RobotState.getInstance().getTeam()) {
            // Red side
            setLaunchPoint = new Pose2d(1.22, 0, new Rotation2d(2.58309));
        } else {
            // Blue side
            setLaunchPoint = new Pose2d(1.22, 0, new Rotation2d(-2.68781));
        }

//        tele.addData("What index for sorter: ", sorter.getCurrentIndex());
//        tele.addData("Color for sorter: ", Arrays.toString(RobotState.getInstance().getBallColors()));
//        tele.addData("Team isBlue", RobotState.getInstance().getTeam());
//        tele.addData("Heading offset", RobotState.getInstance().getHeadingOffset());
//        tele.update();
    }

    private void assignControls() {
        // Set up continuous drive
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

        intakeLeft.setDefaultCommand(new RunLeftIntakeContinuous(intakeLeft, .2));
        intakeRight.setDefaultCommand(new RunRightIntakeContinuous(intakeRight, .2));

//        turretRotate.setDefaultCommand(new TurretRotateContinuous(turretRotate));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1)
                .whenActive(new RunRightIntakeContinuous(intakeRight, IntakeLeft.INTAKE_POWER))
                .whenInactive(new RunRightIntake(intakeRight, 0));

        driver.getGamepadButton(RIGHT_BUMPER)
                .whenPressed(new RunRightIntakeContinuous(intakeRight, IntakeLeft.OUTTAKE_POWER))
                .whenReleased(new RunRightIntake(intakeRight, 0));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1)
                .whenActive(new RunLeftIntakeContinuous(intakeLeft, IntakeLeft.INTAKE_POWER))
                .whenInactive(new RunLeftIntake(intakeLeft, 0));

        driver.getGamepadButton(LEFT_BUMPER)
                .whenPressed(new RunLeftIntakeContinuous(intakeLeft, IntakeLeft.OUTTAKE_POWER))
                .whenReleased(new RunLeftIntake(intakeLeft, 0));

        driver.getGamepadButton(A)
                .whenPressed(new TestLaunchSequence(feederServo, feederMotor, intakeRight)
                );
//
        driver.getGamepadButton(B)
                .whenPressed(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> turretRotate.resetEncoder())));

//        driver.getGamepadButton(B)
//                .whenPressed(
//                        new ParallelCommandGroup(
//                                new LuanchSetPattern(launcher, sorter, holdControl, patters.get(22))));
//
//        driver.getGamepadButton(Y)
//                .whenPressed(
//                        new ParallelCommandGroup(
//                                new LuanchSetPattern(launcher, sorter, holdControl, patters.get(23))));
//
//        driver.getGamepadButton(X)
//                        .whenPressed(new DynamicStrafeCommand(drivetrain, () -> setLaunchPoint));
//
//        driver.getGamepadButton(START)
//                        .whenPressed(new SetLocalizerHeading(drivetrain, 0));
//
//        driver.getGamepadButton(BACK)
//                        .whenPressed(new GetTagPattern(vision));

//        driver.getGamepadButton(A)
//                        .whenPressed(new HoldControlCommand(holdControl, HoldControl.HoldPosition.LAUNCHING));
//
//        driver.getGamepadButton(B)
//                .whenPressed(new HoldControlCommand(holdControl, HoldControl.HoldPosition.TRANSPORT));

        driver.getGamepadButton(DPAD_UP)
                .whenPressed(new ParallelCommandGroup(
                        new SetPowerLauncher(launcher, 1)
                ));
//
//        driver.getGamepadButton(DPAD_LEFT)
//                        .whenPressed(
//                                new ParallelCommandGroup(
//                                        new InstantCommand(launcher::enable),
//                                        new InstantCommand(launcher::getAndSetFlywheelByDistance),
//                                        new SetHoodAngleCommand(hood, 1430),
//                                        new TurnToTarget(drivetrain, driver, 1, vision)
//                                )
//                        );

//        driver.getGamepadButton(DPAD_RIGHT)
//                        .whenPressed(new ParallelCommandGroup(
//                                new InstantCommand(() -> drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1)))
//                        ));


        driver.getGamepadButton(DPAD_DOWN)
                .whenPressed(
                        new ParallelCommandGroup(
                                new StopFlywheel(launcher)
                ));
    }
}

