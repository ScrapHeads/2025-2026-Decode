package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.util.ConversionUtil.convertChassisSpeeds;
import static org.firstinspires.ftc.teamcode.util.ConversionUtil.convertPose2D;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Transform2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Units;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

import java.util.Arrays;
import java.util.List;

/**
 * Subsystem for controlling a mecanum drivetrain using Road Runner.
 * <p>
 * Responsibilities:
 * <ul>
 *   <li>Hardware initialization (motors, IMU, voltage sensor).</li>
 *   <li>Localization using Road Runner localizer classes.</li>
 *   <li>Trajectory following and turn actions via Road Runner {@link Action}.</li>
 *   <li>Pose history logging and telemetry visualization on FTC Dashboard.</li>
 *   <li>Provides default motion constraints and trajectory builders.</li>
 * </ul>
 */
@Config
public final class Drivetrain implements Subsystem {
    /**
     * Tunable parameters for kinematics, feedforward, path/turn profiles, and controller gains.
     * Modify through FTC Dashboard during testing.
     */
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        public double inPerTick = 0.00197222585;
        public double lateralInPerTick = 0.0013198824301367726;
        public double trackWidthTicks = 7018.915898072283;

        // feedforward parameters (in tick units)
        public double kS = 1.0314389076602932;
        public double kV =  0.000307111074021038;
        public double kA = 0.00005;

        // path profile parameters (in inches)
        public double maxWheelVel = 60;
        public double minProfileAccel = -40;
        public double maxProfileAccel = 60;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 1;
        public double lateralGain = 1;
        public double headingGain = 1; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn

        public double defaultPosTolerance = .1;
        public double defaultHeadingTolerance = Math.toRadians(2);
        public double defaultVelTolerance = .5;
    }

    public static Params PARAMS = new Params();

    // Kinematics and constraints
    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);

    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));

    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    // Hardware references
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final VoltageSensor voltageSensor;
    public final LazyImu lazyImu;

    // Localization
    public final Localizer localizer;

    // Log writers
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    /**
     * Creates a new drivetrain subsystem.
     *
     * @param hardwareMap FTC hardware map
     * @param pose        initial starting pose estimate
     */
    public Drivetrain(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
               PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(ChassisSpeeds speeds) {
        PoseVelocity2d powers = convertChassisSpeeds(speeds);
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    /**
     * Road Runner Action for following a time-parameterized trajectory.
     */
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = convertPose2D(t.path.get(disps.get(i), 1).value());
                xPoints[i] = p.getX();
                yPoints[i] = p.getY();
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            ChassisSpeeds robotVelRobot = updatePoseEstimate();

            // NOTE: if you don't want vision to impact movement, use odometry pose instead
            Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, convertPose2D(estimatedPose), convertChassisSpeeds(robotVelRobot));
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", estimatedPose.getX());
            p.put("y", estimatedPose.getY());
            p.put("heading (deg)", estimatedPose.getRotation().toString());

            Transform2d error = convertPose2D(txWorldTarget.value()).minus(estimatedPose);
            p.put("xError", error.getX());
            p.put("yError", error.getY());
            p.put("headingError (deg)", error.getRotation().toString());

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, convertPose2D(estimatedPose));

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    /**
     * Road Runner Action for performing a timed turn.
     */
    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            ChassisSpeeds robotVelRobot = updatePoseEstimate();

            // NOTE: if you don't want vision to impact movement, use odometry pose instead
            Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, convertPose2D(estimatedPose), convertChassisSpeeds(robotVelRobot));
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, convertPose2D(estimatedPose));

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    /**
     * Updates the pose estimate and records it to history/log.
     *
     * @return current velocity estimate
     */
    public ChassisSpeeds updatePoseEstimate() {
        ChassisSpeeds vel = localizer.update();

        RobotState.getInstance().addOdometryObservation(localizer.getPose(), TimeTracker.getTime());
        RobotState.getInstance().setChassisSpeeds(vel);

        estimatedPoseWriter.write(new PoseMessage(convertPose2D(RobotState.getInstance().getOdometryPose())));

        return vel;
    }

    /**
     * Draws the robot's pose history on the dashboard field overlay.
     */
    private void drawPoseHistory(Canvas c) {
//        double[] xPoints = new double[poseHistory.size()];
//        double[] yPoints = new double[poseHistory.size()];
//
//        int i = 0;
//        for (Pose2d t : poseHistory) {
//            xPoints[i] = t.position.x;
//            yPoints[i] = t.position.y;
//
//            i++;
//        }
//
//        c.setStrokeWidth(1);
//        c.setStroke("#3F51B5");
//        c.strokePolyline(xPoints, yPoints);
    }

    /**
     * Builds a trajectory action starting from a given pose using drivetrain defaults.
     *
     * @param beginPose the starting pose of the trajectory
     * @return a new {@link TrajectoryActionBuilder} with default tolerances and constraints
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                PARAMS.defaultPosTolerance, PARAMS.defaultHeadingTolerance, PARAMS.defaultVelTolerance
                        )
                ),
                convertPose2D(beginPose), 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }


    /**
     * Builds a trajectory action starting from a given pose with custom tolerances.
     *
     * @param beginPose  the starting pose of the trajectory
     * @param posTol     position tolerance (inches)
     * @param headingTol heading tolerance (radians)
     * @param velTol     velocity tolerance (inches/sec or rad/sec)
     * @return a new {@link TrajectoryActionBuilder} with default constraints and custom tolerances
     */
    public TrajectoryActionBuilder actionBuilder(
            Pose2d beginPose,
            double posTol, double headingTol, double velTol
    ) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(posTol, headingTol, velTol)
                ),
                convertPose2D(beginPose), 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    /**
     * Builds a trajectory action starting from a given pose with custom motion constraints
     * but default tolerances.
     *
     * @param beginPose       the starting pose of the trajectory
     * @param turnConstraints constraints on angular velocity and acceleration
     * @param velConstraints  linear velocity constraint
     * @param accelConstraint linear acceleration constraint
     * @return a new {@link TrajectoryActionBuilder} with custom constraints and default tolerances
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose, TurnConstraints turnConstraints, VelConstraint velConstraints, AccelConstraint accelConstraint) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.1, Math.toRadians(.5), .00005
                        )
                ),
                convertPose2D(beginPose), 0.0,
                turnConstraints,
                velConstraints, accelConstraint
        );
    }

    /**
     * Builds a trajectory action starting from a given pose with full custom constraints and tolerances.
     *
     * @param beginPose       the starting pose of the trajectory
     * @param turnConstraints constraints on angular velocity and acceleration
     * @param velConstraints  linear velocity constraint
     * @param accelConstraint linear acceleration constraint
     * @param posTol          position tolerance (inches)
     * @param headingTol      heading tolerance (radians)
     * @param velTol          velocity tolerance (inches/sec or rad/sec)
     * @return a new {@link TrajectoryActionBuilder} with full custom parameters
     */
    public TrajectoryActionBuilder actionBuilder(
            Pose2d beginPose,
            TurnConstraints turnConstraints,
            VelConstraint velConstraints,
            AccelConstraint accelConstraint,
            double posTol, double headingTol, double velTol
    ) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(posTol, headingTol, velTol)
                ),
                convertPose2D(beginPose), 0.0,
                turnConstraints,
                velConstraints, accelConstraint
        );
    }

    /**
     * Periodically updates localization and sends telemetry to dashboard.
     */
    @Override
    public void periodic() {
        ChassisSpeeds vel = updatePoseEstimate();
        Pose2d pose = RobotState.getInstance().getOdometryPose();

        TelemetryPacket packet = new TelemetryPacket();
        // Use this when changing pos tracking to meters
        // Units.metersToInches()
        packet.put("X", pose.getX());
        packet.put("Y", pose.getY());
        packet.put("Localizer pose", localizer.getPose().toString());
        packet.put("rot", pose.getRotation().toString());
        packet.put("xVel", vel.vxMetersPerSecond);
        packet.put("yVel", vel.vyMetersPerSecond);
        packet.put("rotVel", vel.omegaRadiansPerSecond);

        Drawing.drawRobot(packet.fieldOverlay(), convertPose2D(pose));

        dashboard.sendTelemetryPacket(packet);
    }
}