package org.firstinspires.ftc.teamcode.teleops;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.Constants.turretLookupTable;
import static org.firstinspires.ftc.teamcode.util.BallColor.EMPTY;

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
import org.firstinspires.ftc.teamcode.Commands.feeder.RunFeederMotor;
import org.firstinspires.ftc.teamcode.Commands.feeder.RunFeederMotorContinuous;
import org.firstinspires.ftc.teamcode.Commands.feeder.TurnFeederServoBothContinuous;
import org.firstinspires.ftc.teamcode.Commands.intake.RunLeftIntake;
import org.firstinspires.ftc.teamcode.Commands.intake.RunLeftIntakeContinuous;
import org.firstinspires.ftc.teamcode.Commands.intake.RunRightIntake;
import org.firstinspires.ftc.teamcode.Commands.intake.RunRightIntakeContinuous;
import org.firstinspires.ftc.teamcode.Commands.launcher.launchsequence.LaunchSequenceLeft;
import org.firstinspires.ftc.teamcode.Commands.launcher.launchsequence.LaunchSequenceRight;
import org.firstinspires.ftc.teamcode.Commands.launcher.SetRpmDistanceContinuous;
import org.firstinspires.ftc.teamcode.Commands.launcher.StopFlywheel;
import org.firstinspires.ftc.teamcode.Commands.stopper.TurnStopperBoth;
import org.firstinspires.ftc.teamcode.Commands.turret.TurretRotateContinuous;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.TurretLookupTable;
import org.firstinspires.ftc.teamcode.subsystems.intake.BallStoppers;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FeederMotor;
import org.firstinspires.ftc.teamcode.subsystems.FeederServo;
import org.firstinspires.ftc.teamcode.subsystems.turret.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHood;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretRotate;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeLeft;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeRight;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

@TeleOp(name = "PracticeAllSystemsTeleRed", group = "ScrapHeads")
public class PracticeAllSystemsTeleRed extends CommandOpMode {
    // Controller
    private GamepadEx driver;

    // Subsystem
    private Drivetrain drivetrain;
    private Launcher launcher;
    private IntakeLeft intakeLeft;
    private IntakeRight intakeRight;
    private TurretHood turretHood;
    private Vision vision;
    private FeederServo feederServo;
    private FeederMotor feederMotor;
    private TurretRotate turretRotate;
    private BallStoppers ballStoppers;

    private enum DriveStates {
        DEFAULT,
        SLOW
    }

    private DriveStates driveStates = DriveStates.DEFAULT;

    @Override
    public void initialize() {
        // Initialize shared constants
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();
        turretLookupTable = new TurretLookupTable();

//        StateIO.load();

        setUpRobotState();

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

        // Uncomment when testing auto first
        turretRotate.resetEncoder();

        ballStoppers = new BallStoppers(hm);
        ballStoppers.register();

        turretHood = new TurretHood(hm);
        turretHood.register();

        vision = new Vision(hm);
        vision.register();

        turretRotate.enablePID();

        // Bind controls
        assignControls();

        CommandScheduler.getInstance().cancelAll();

        vision.setPipeline(0);

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

        turretRotate.setDefaultCommand(new TurretRotateContinuous(turretRotate));

        launcher.setDefaultCommand(new SetRpmDistanceContinuous(launcher));

        feederMotor.setDefaultCommand(new RunFeederMotorContinuous(feederMotor, 0));
        feederServo.setDefaultCommand(new TurnFeederServoBothContinuous(
                feederServo, FeederServo.IN_FRONT_ANGLE, FeederServo.IN_BACK_ANGLE));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1)
                .whenActive(
                        new ParallelCommandGroup(
                                new RunRightIntakeContinuous(intakeRight, IntakeLeft.INTAKE_POWER),
                                new TurnStopperBoth(ballStoppers, BallStoppers.DOWN_ANGLE_LEFT, BallStoppers.UP_ANGLE_RIGHT)
                        ))
                .whenInactive(new RunRightIntake(intakeRight, 0));

        driver.getGamepadButton(RIGHT_BUMPER)
                .whenPressed(new RunRightIntakeContinuous(intakeRight, IntakeLeft.OUTTAKE_POWER))
                .whenReleased(new RunRightIntake(intakeRight, 0));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1)
                .whenActive(
                        new ParallelCommandGroup (
                                new RunLeftIntakeContinuous(intakeLeft, IntakeLeft.INTAKE_POWER),
                                new TurnStopperBoth(ballStoppers, BallStoppers.UP_ANGLE_LEFT, BallStoppers.DOWN_ANGLE_RIGHT)
                        )
                )
                .whenInactive(new RunLeftIntake(intakeLeft, 0));

        driver.getGamepadButton(LEFT_BUMPER)
                .whenPressed(new RunLeftIntakeContinuous(intakeLeft, IntakeLeft.OUTTAKE_POWER))
                .whenReleased(new RunLeftIntake(intakeLeft, 0));

        driver.getGamepadButton(X)
                .whenPressed(new LaunchSequenceLeft(feederServo, feederMotor, ballStoppers, intakeLeft, turretRotate, launcher));

        driver.getGamepadButton(B)
                .whenPressed(new LaunchSequenceRight(feederServo, feederMotor, ballStoppers, intakeRight, turretRotate));

//        driver.getGamepadButton(START)
//                .whenPressed(
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> turretRotate.resetEncoder())));

        driver.getGamepadButton(DPAD_UP)
                .whenPressed(new InstantCommand(() -> launcher.enable()));

        driver.getGamepadButton(DPAD_DOWN)
                .whenPressed(new StopFlywheel(launcher));
//
        driver.getGamepadButton(DPAD_LEFT)
                .whenPressed(
                        new RunFeederMotor(feederMotor, FeederMotor.DOWN_POWER)
                ).whenReleased(
                        new RunFeederMotor(feederMotor, 0)
                );

        driver.getGamepadButton(DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> turretRotate.disablePID()));

        driver.getGamepadButton(BACK)
                .whenPressed(new InstantCommand(this::advanceDriveStates));

        new Trigger(() -> driveStates == DriveStates.SLOW)
                .whenActive(new DriveContinous(drivetrain, driver, .3).interruptOn(() -> driveStates == DriveStates.DEFAULT));
    }
    private void advanceDriveStates () {
        switch (driveStates) {
            case DEFAULT:
                driveStates = DriveStates.SLOW;
                break;
            case SLOW:
                driveStates = DriveStates.DEFAULT;
                break;
        }
    }

    public void setUpRobotState() {
        RobotState.getInstance().addOdometryObservation(new Pose2d(), TimeTracker.getTime());

        RobotState.getInstance().setTeam(false);

        RobotState.getInstance().setPattern(new BallColor[] {EMPTY, EMPTY, EMPTY});
    }
}

