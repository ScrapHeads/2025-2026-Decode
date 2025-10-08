// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.RilLib.Math.Geometry;

import org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation.Interpolatable;
import org.firstinspires.ftc.teamcode.RilLib.Math.MatBuilder;
import org.firstinspires.ftc.teamcode.RilLib.Math.Matrix;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N4;
import org.firstinspires.ftc.teamcode.RilLib.Math.Util.Pose3dUtil;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.RilLib.Math.Vector;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Objects;

/** Represents a 3D pose containing translational and rotational elements. */
public class Pose3d implements Interpolatable<Pose3d> {
    /**
     * A preallocated Pose3d representing the origin.
     *
     * <p>
     * This exists to avoid allocations for common poses.
     */
    public static final Pose3d kZero = new Pose3d();

    private final Translation3d m_translation;
    private final Rotation3d m_rotation;

    /** Constructs a pose at the origin facing toward the positive X axis. */
    public Pose3d() {
        m_translation = Translation3d.kZero;
        m_rotation = Rotation3d.kZero;
    }

    /**
     * Constructs a pose with the specified translation and rotation.
     *
     * @param translation The translational component of the pose.
     * @param rotation    The rotational component of the pose.
     */
    public Pose3d(Translation3d translation, Rotation3d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /**
     * Constructs a pose with x, y, and z translations instead of a separate
     * Translation3d.
     *
     * @param x        The x component of the translational component of the pose.
     * @param y        The y component of the translational component of the pose.
     * @param z        The z component of the translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    public Pose3d(double x, double y, double z, Rotation3d rotation) {
        m_translation = new Translation3d(x, y, z);
        m_rotation = rotation;
    }

    /**
     * Constructs a pose with the specified affine transformation matrix.
     *
     * @param matrix The affine transformation matrix.
     * @throws IllegalArgumentException if the affine transformation matrix is
     *                                  invalid.
     */
    public Pose3d(Matrix<N4, N4> matrix) {
        m_translation = new Translation3d(matrix.get(0, 3), matrix.get(1, 3), matrix.get(2, 3));
        m_rotation = new Rotation3d(matrix.block(3, 3, 0, 0));
        if (matrix.get(3, 0) != 0.0
                || matrix.get(3, 1) != 0.0
                || matrix.get(3, 2) != 0.0
                || matrix.get(3, 3) != 1.0) {
            throw new IllegalArgumentException("Affine transformation matrix is invalid");
        }
    }

    /**
     * Constructs a 3D pose from a 2D pose in the X-Y plane.
     *
     * @param pose The 2D pose.
     * @see Rotation3d#Rotation3d(Rotation2d)
     * @see Translation3d#Translation3d(Translation2d)
     */
    public Pose3d(Pose2d pose) {
        m_translation = new Translation3d(pose.getX(), pose.getY(), 0.0);
        m_rotation = new Rotation3d(0.0, 0.0, pose.getRotation().getRadians());
    }

    /**
     * Transforms the pose by the given transformation and returns the new
     * transformed pose. The
     * transform is applied relative to the pose's frame. Note that this differs
     * from {@link
     * Pose3d#rotateBy(Rotation3d)}, which is applied relative to the global frame
     * and around the
     * origin.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public Pose3d plus(Transform3d other) {
        return transformBy(other);
    }

    /**
     * Returns the Transform3d that maps the one pose to another.
     *
     * @param other The initial pose of the transformation.
     * @return The transform that maps the other pose to the current pose.
     */
    public Transform3d minus(Pose3d other) {
        final Pose3d pose = this.relativeTo(other);
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Returns the translation component of the transformation.
     *
     * @return The translational component of the pose.
     */
    public Translation3d getTranslation() {
        return m_translation;
    }

    /**
     * Returns the X component of the pose's translation.
     *
     * @return The x component of the pose's translation.
     */
    public double getX() {
        return m_translation.getX();
    }

    /**
     * Returns the Y component of the pose's translation.
     *
     * @return The y component of the pose's translation.
     */
    public double getY() {
        return m_translation.getY();
    }

    /**
     * Returns the Z component of the pose's translation.
     *
     * @return The z component of the pose's translation.
     */
    public double getZ() {
        return m_translation.getZ();
    }

    /**
     * Returns the rotational component of the transformation.
     *
     * @return The rotational component of the pose.
     */
    public Rotation3d getRotation() {
        return m_rotation;
    }

    /**
     * Multiplies the current pose by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Pose3d.
     */
    public Pose3d times(double scalar) {
        return new Pose3d(m_translation.times(scalar), m_rotation.times(scalar));
    }

    /**
     * Divides the current pose by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Pose3d.
     */
    public Pose3d div(double scalar) {
        return times(1.0 / scalar);
    }

    /**
     * Rotates the pose around the origin and returns the new pose.
     *
     * @param other The rotation to transform the pose by, which is applied
     *              extrinsically (from the
     *              global frame).
     * @return The rotated pose.
     */
    public Pose3d rotateBy(Rotation3d other) {
        return new Pose3d(m_translation.rotateBy(other), m_rotation.rotateBy(other));
    }

    /**
     * Transforms the pose by the given transformation and returns the new
     * transformed pose. The
     * transform is applied relative to the pose's frame. Note that this differs
     * from {@link
     * Pose3d#rotateBy(Rotation3d)}, which is applied relative to the global frame
     * and around the
     * origin.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public Pose3d transformBy(Transform3d other) {
        return new Pose3d(
                m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
                other.getRotation().plus(m_rotation));
    }

    /**
     * Returns the current pose relative to the given pose.
     *
     * <p>
     * This function can often be used for trajectory tracking or pose stabilization
     * algorithms to
     * get the error between the reference and the current pose.
     *
     * @param other The pose that is the origin of the new coordinate frame that the
     *              current pose will
     *              be converted into.
     * @return The current pose relative to the new origin pose.
     */
    public Pose3d relativeTo(Pose3d other) {
        Transform3d transform = new Transform3d(other, this);
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Rotates the current pose around a point in 3D space.
     *
     * @param point The point in 3D space to rotate around.
     * @param rot   The rotation to rotate the pose by.
     * @return The new rotated pose.
     */
    public Pose3d rotateAround(Translation3d point, Rotation3d rot) {
        return new Pose3d(m_translation.rotateAround(point, rot), m_rotation.rotateBy(rot));
    }

    /**
     * Obtain a new Pose3d from a (constant curvature) velocity.
     *
     * <p>
     * The twist is a change in pose in the robot's coordinate frame since the
     * previous pose
     * update. When the user runs exp() on the previous known field-relative pose
     * with the argument
     * being the twist, the user will receive the new field-relative pose.
     *
     * <p>
     * "Exp" represents the pose exponential, which is solving a differential
     * equation moving the
     * pose forward in time.
     *
     * @param twist The change in pose in the robot's coordinate frame since the
     *              previous pose update.
     *              For example, if a non-holonomic robot moves forward 0.01 meters
     *              and changes angle by 0.5
     *              degrees since the previous pose update, the twist would be
     *              Twist3d(0.01, 0.0, 0.0, new
     *              Rotation3d(0.0, 0.0, Units.degreesToRadians(0.5))).
     * @return The new pose of the robot.
     */
    public Pose3d exp(Twist3d twist) {
        // Current Pose
        Quaternion currentQuat = this.getRotation().getQuaternion();

        // Twist components
        double[] u = { twist.dx, twist.dy, twist.dz };
        double[] rvec = { twist.rx, twist.ry, twist.rz };

        double[][] omega = Pose3dUtil.omegaFromVector(rvec);
        double[][] omegaSq = Pose3dUtil.matMul(omega, omega);

        double theta = Pose3dUtil.vecNorm(rvec);
        double thetaSq = theta * theta;

        double A;
        double B;
        double C;
        if (Math.abs(theta) < 1E-7) {
            // Taylor Expansions around θ = 0
            // A = 1/1! - θ²/3! + θ⁴/5!
            // B = 1/2! - θ²/4! + θ⁴/6!
            // C = 1/3! - θ²/5! + θ⁴/7!

            A = 1 - thetaSq / 6 + thetaSq * thetaSq / 120;
            B = 1 / 2.0 - thetaSq / 24 + thetaSq * thetaSq / 720;
            C = 1 / 6.0 - thetaSq / 120 + thetaSq * thetaSq / 5040;
        } else {
            // A = sin(θ)/θ
            // B = (1 - cos(θ)) / θ²
            // C = (1 - A) / θ²
            A = Math.sin(theta) / theta;
            B = (1 - Math.cos(theta)) / thetaSq;
            C = (1 - A) / thetaSq;
        }

        // V = I + B*omega + C*omegaSq
        double[][] V = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                V[i][j] = (i == j ? 1.0 : 0.0) + B * omega[i][j] + C * omegaSq[i][j];
            }
        }

        double[] transComp = Pose3dUtil.matVecMul(V, u);

        // Rotation from rotation vector
        Vector<N3> rvecVec = VecBuilder.fill(rvec[0], rvec[1], rvec[2]);
        Quaternion otherQuat = Quaternion.fromRotationVector(rvecVec);

        Quaternion resultQuat = otherQuat.times(currentQuat);

        // Rotate translation_component by current rotation and add to current
        // translation
        Translation3d transComponentTranslation = new Translation3d(transComp[0], transComp[1], transComp[2]);
        double resX = m_translation.getX() + transComponentTranslation.getX();
        double resY = m_translation.getY() + transComponentTranslation.getY();
        double resZ = m_translation.getZ() + transComponentTranslation.getZ();

        return new Pose3d(
                resX,
                resY,
                resZ,
                new Rotation3d(resultQuat));
    }

    /**
     * Returns a Twist3d that maps this pose to the end pose. If c is the output of
     * {@code a.Log(b)},
     * then {@code a.Exp(c)} would yield b.
     *
     * @param end The end pose for the transformation.
     * @return The twist that maps this to end.
     */
    public Twist3d log(Pose3d end) {
        Pose3d transform = end.relativeTo(this);
        double[] u = { transform.getTranslation().getX(), transform.getTranslation().getY(),
                transform.getTranslation().getZ() };

        Vector<N3> rvecVec = transform.getRotation().toVector();
        double[] rvec = { rvecVec.get(0), rvecVec.get(1), rvecVec.get(2) };

        double[][] omega = Pose3dUtil.omegaFromVector(rvec);
        double[][] omegaSq = Pose3dUtil.matMul(omega, omega);

        double theta = Pose3dUtil.vecNorm(rvec);
        double thetaSq = theta * theta;

        double C;
        if (Math.abs(theta) < 1E-7) {
            // Taylor Expansions around θ = 0
            // A = 1/1! - θ²/3! + θ⁴/5!
            // B = 1/2! - θ²/4! + θ⁴/6!
            // C = 1/6 * (1/2 + θ²/5! + θ⁴/7!)

            C = 1 / 12.0 - thetaSq / 720 + thetaSq * thetaSq / 30240;
        } else {
            // A = sin(θ)/θ
            // B = (1 - cos(θ)) / θ²
            // C = (1 - A/(2*B)) / θ²
            double A = Math.sin(theta) / theta;
            double B = (1 - Math.cos(theta)) / thetaSq;
            C = (1 - A / (2 * B)) / thetaSq;
        }

        // V_inv = I - 0.5*omega + C*omegaSq
        double[][] Vinv = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Vinv[i][j] = (i == j ? 1.0 : 0.0) - 0.5 * omega[i][j] + C * omegaSq[i][j];
            }
        }

        double[] transComp = Pose3dUtil.matVecMul(Vinv, u);

        return new Twist3d(
                transComp[0],
                transComp[1],
                transComp[2],
                rvec[0],
                rvec[1],
                rvec[2]);
    }

    /**
     * Returns an affine transformation matrix representation of this pose.
     *
     * @return An affine transformation matrix representation of this pose.
     */
    public Matrix<N4, N4> toMatrix() {
        Vector<N3> vec = m_translation.toVector();
        Matrix<N3, N3> mat = m_rotation.toMatrix();
        return MatBuilder.fill(
                N4.instance,
                N4.instance,
                mat.get(0, 0),
                mat.get(0, 1),
                mat.get(0, 2),
                vec.get(0),
                mat.get(1, 0),
                mat.get(1, 1),
                mat.get(1, 2),
                vec.get(1),
                mat.get(2, 0),
                mat.get(2, 1),
                mat.get(2, 2),
                vec.get(2),
                0.0,
                0.0,
                0.0,
                1.0);
    }

    /**
     * Returns a Pose2d representing this Pose3d projected into the X-Y plane.
     *
     * @return A Pose2d representing this Pose3d projected into the X-Y plane.
     */
    public Pose2d toPose2d() {
        return new Pose2d(m_translation.toTranslation2d(), m_rotation.toRotation2d());
    }

    /**
     * Returns the nearest Pose3d from a collection of poses. If two or more poses
     * in the collection
     * have the same distance from this pose, return the one with the closest
     * rotation component.
     *
     * @param poses The collection of poses to find the nearest.
     * @return The nearest Pose3d from the collection.
     */
    public Pose3d nearest(Collection<Pose3d> poses) {
        return Collections.min(
                poses,
                Comparator.comparing(
                        (Pose3d other) -> this.getTranslation().getDistance(other.getTranslation()))
                        .thenComparing(
                                (Pose3d other) -> this.getRotation().minus(other.getRotation()).getAngle()));
    }

    @Override
    public String toString() {
        return String.format("Pose3d(%s, %s)", m_translation, m_rotation);
    }

    /**
     * Checks equality between this Pose3d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Pose3d) {
            Pose3d pose = (Pose3d) obj;
            return m_translation.equals(pose.m_translation)
                    && m_rotation.equals(pose.m_rotation);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_translation, m_rotation);
    }

    @Override
    public Pose3d interpolate(Pose3d endValue, double t) {
        if (t < 0) {
            return this;
        } else if (t >= 1) {
            return endValue;
        } else {
            Twist3d twist = this.log(endValue);
            Twist3d scaledTwist = new Twist3d(
                    twist.dx * t, twist.dy * t, twist.dz * t, twist.rx * t, twist.ry * t, twist.rz * t);
            return this.exp(scaledTwist);
        }
    }
}