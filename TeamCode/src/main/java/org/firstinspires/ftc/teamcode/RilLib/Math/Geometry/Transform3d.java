// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.RilLib.Math.Geometry;

import org.firstinspires.ftc.teamcode.RilLib.Math.MatBuilder;
import org.firstinspires.ftc.teamcode.RilLib.Math.Matrix;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N4;
import org.firstinspires.ftc.teamcode.RilLib.Math.Vector;

import java.util.Objects;

/** Represents a transformation for a Pose3d in the pose's frame. */
public class Transform3d {
    /**
     * A preallocated Transform3d representing no transformation.
     *
     * <p>
     * This exists to avoid allocations for common transformations.
     */
    public static final Transform3d kZero = new Transform3d();

    private final Translation3d m_translation;
    private final Rotation3d m_rotation;

    /**
     * Constructs the transform that maps the initial pose to the final pose.
     *
     * @param initial The initial pose for the transformation.
     * @param last    The final pose for the transformation.
     */
    public Transform3d(Pose3d initial, Pose3d last) {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        m_translation = last.getTranslation()
                .minus(initial.getTranslation())
                .rotateBy(initial.getRotation().unaryMinus());

        m_rotation = last.getRotation().minus(initial.getRotation());
    }

    /**
     * Constructs a transform with the given translation and rotation components.
     *
     * @param translation Translational component of the transform.
     * @param rotation    Rotational component of the transform.
     */
    public Transform3d(Translation3d translation, Rotation3d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /**
     * Constructs a transform with x, y, and z translations instead of a separate
     * Translation3d.
     *
     * @param x        The x component of the translational component of the
     *                 transform.
     * @param y        The y component of the translational component of the
     *                 transform.
     * @param z        The z component of the translational component of the
     *                 transform.
     * @param rotation The rotational component of the transform.
     */
    public Transform3d(double x, double y, double z, Rotation3d rotation) {
        m_translation = new Translation3d(x, y, z);
        m_rotation = rotation;
    }

    /**
     * Constructs a transform with the specified affine transformation matrix.
     *
     * @param matrix The affine transformation matrix.
     * @throws IllegalArgumentException if the affine transformation matrix is
     *                                  invalid.
     */
    public Transform3d(Matrix<N4, N4> matrix) {
        m_translation = new Translation3d(matrix.get(0, 3), matrix.get(1, 3), matrix.get(2, 3));
        m_rotation = new Rotation3d(matrix.block(3, 3, 0, 0));
        if (matrix.get(3, 0) != 0.0
                || matrix.get(3, 1) != 0.0
                || matrix.get(3, 2) != 0.0
                || matrix.get(3, 3) != 1.0) {
            throw new IllegalArgumentException("Affine transformation matrix is invalid");
        }
    }

    /** Constructs the identity transform -- maps an initial pose to itself. */
    public Transform3d() {
        m_translation = Translation3d.kZero;
        m_rotation = Rotation3d.kZero;
    }

    /**
     * Constructs a 3D transform from a 2D transform in the X-Y plane.
     *
     * @param transform The 2D transform.
     * @see Rotation3d#Rotation3d(Rotation2d)
     * @see Translation3d#Translation3d(Translation2d)
     */
    public Transform3d(Transform2d transform) {
        m_translation = new Translation3d(transform.getTranslation());
        m_rotation = new Rotation3d(transform.getRotation());
    }

    /**
     * Multiplies the transform by the scalar.
     *
     * @param scalar The scalar.
     * @return The scaled Transform3d.
     */
    public Transform3d times(double scalar) {
        return new Transform3d(m_translation.times(scalar), m_rotation.times(scalar));
    }

    /**
     * Divides the transform by the scalar.
     *
     * @param scalar The scalar.
     * @return The scaled Transform3d.
     */
    public Transform3d div(double scalar) {
        return times(1.0 / scalar);
    }

    /**
     * Composes two transformations. The second transform is applied relative to the
     * orientation of
     * the first.
     *
     * @param other The transform to compose with this one.
     * @return The composition of the two transformations.
     */
    public Transform3d plus(Transform3d other) {
        return new Transform3d(Pose3d.kZero, Pose3d.kZero.transformBy(this).transformBy(other));
    }

    /**
     * Returns the translation component of the transformation.
     *
     * @return The translational component of the transform.
     */
    public Translation3d getTranslation() {
        return m_translation;
    }

    /**
     * Returns the X component of the transformation's translation.
     *
     * @return The x component of the transformation's translation.
     */
    public double getX() {
        return m_translation.getX();
    }

    /**
     * Returns the Y component of the transformation's translation.
     *
     * @return The y component of the transformation's translation.
     */
    public double getY() {
        return m_translation.getY();
    }

    /**
     * Returns the Z component of the transformation's translation.
     *
     * @return The z component of the transformation's translation.
     */
    public double getZ() {
        return m_translation.getZ();
    }

    /**
     * Returns an affine transformation matrix representation of this
     * transformation.
     *
     * @return An affine transformation matrix representation of this
     *         transformation.
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
     * Returns the rotational component of the transformation.
     *
     * @return Reference to the rotational component of the transform.
     */
    public Rotation3d getRotation() {
        return m_rotation;
    }

    /**
     * Invert the transformation. This is useful for undoing a transformation.
     *
     * @return The inverted transformation.
     */
    public Transform3d inverse() {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        return new Transform3d(
                getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()),
                getRotation().unaryMinus());
    }

    @Override
    public String toString() {
        return String.format("Transform3d(%s, %s)", m_translation, m_rotation);
    }

    /**
     * Checks equality between this Transform3d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Transform3d) {
            Transform3d other = (Transform3d) obj;
            return other.m_translation.equals(m_translation)
                    && other.m_rotation.equals(m_rotation);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_translation, m_rotation);
    }
}