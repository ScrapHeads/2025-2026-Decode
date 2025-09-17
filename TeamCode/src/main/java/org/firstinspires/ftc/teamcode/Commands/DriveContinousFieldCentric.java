package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveContinousFieldCentric extends CommandBase {
    private Drivetrain drivetrain;
    private GamepadEx driver;
    private double speed;

    public DriveContinousFieldCentric(Drivetrain drivetrain, GamepadEx driver, double speed) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.speed = speed;

        addRequirements(drivetrain);
    }

//    @Override
//    public void initialize(){
//
//    }

    @Override
    public void execute(){
        // 1. Get the robot's current heading in radians
        double heading = drivetrain.localizer.getPose().heading.toDouble();

        // 2. Read joystick inputs (Y forward, X right, Rx rotation)
        double yInput = driver.getLeftY() * speed;   // forward/back
        double xInput = -driver.getLeftX() * speed;  // strafe (negated to match your original)
        double turnInput = -driver.getRightX() * speed; // rotation (negated to match your original)

        // 3. Rotate the joystick translation vector by -heading (field-centric)
        double rotatedX = xInput * Math.cos(heading) - yInput * Math.sin(heading);
        double rotatedY = xInput * Math.sin(heading) + yInput * Math.cos(heading);

        // 4. Create a field-centric pose velocity
        PoseVelocity2d pose = new PoseVelocity2d(
                new Vector2d(rotatedX, rotatedY), // Road Runner's PoseVelocity2d expects (forward, left)
                turnInput
        );

        // 5. Send to drivetrain
        drivetrain.setDrivePowers(pose);
    }

    @Override
    public void end(boolean isInterrupted){
        PoseVelocity2d pose = new PoseVelocity2d(
                new Vector2d(
                        0,
                        0
                ),
                0);
        drivetrain.setDrivePowers(pose);
    }
}
