package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Transform2d;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 3704.9589480347263; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Transform2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, new Rotation2d());

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;
        //.050213506247
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
//        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);
        driver.setOffsets(-175, 0, DistanceUnit.MM);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = new Transform2d(initialPose.getX(), initialPose.getY(), initialPose.getRotation());
    }

    public void setPose(Pose2d pose) {
        txWorldPinpoint = txPinpointRobot.minus(pose);
    }

    @Override
    public void setPose(com.acmerobotics.roadrunner.Pose2d pose) {
        Pose2d newPose = new Pose2d(pose.position.x, pose.position.y, new Rotation2d(pose.heading.toDouble()));
        txWorldPinpoint = txPinpointRobot.minus(newPose);
    }

    public Pose2d getPose() {
        return txPinpointRobot.plus(txWorldPinpoint);
    }

    public ChassisSpeeds update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(
                    driver.getPosX(DistanceUnit.INCH),
                    driver.getPosY(DistanceUnit.INCH),
                    new Rotation2d(driver.getHeading(UnnormalizedAngleUnit.RADIANS)));

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    driver.getVelX(DistanceUnit.INCH),
                    driver.getVelY(DistanceUnit.INCH),
                    driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS),
                    txPinpointRobot.getRotation());
        }
        return new ChassisSpeeds();
    }
}
