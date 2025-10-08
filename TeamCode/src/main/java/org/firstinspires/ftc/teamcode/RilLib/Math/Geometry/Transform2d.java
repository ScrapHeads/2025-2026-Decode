// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.RilLib.Math.Geometry;

import org.firstinspires.ftc.teamcode.RilLib.Math.MatBuilder;
import org.firstinspires.ftc.teamcode.RilLib.Math.Matrix;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N2;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;
import org.firstinspires.ftc.teamcode.RilLib.Math.Vector;

import java.util.Objects;

/** Represents a transformation for a Pose2d in the pose's frame. */
public class Transform2d {
    /**
     * A preallocated Transform2d representing no transformation.
     *
     * <p>
     * This exists to avoid allocations for common transformations.
     */
    public static final Transform2d kZero = new Transform2d();

    private final Translation2d m_translation;
    private final Rotation2d m_rotation;

    /**
     * Constructs the transform that maps the initial pose to the final pose.
     *
     * @param initial The initial pose for the transformation.
     * @param last    The final pose for the transformation.
     */
    public Transform2d(Pose2d initial, Pose2d last) {
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
    public Transform2d(Translation2d translation, Rotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /**
     * Constructs a transform with x and y translations instead of a separate
     * Translation2d.
     *
     * @param x        The x component of the translational component of the
     *                 transform.
     * @param y        The y component of the translational component of the
     *                 transform.
     * @param rotation The rotational component of the transform.
     */
    public Transform2d(double x, double y, Rotation2d rotation) {
        m_translation = new Translation2d(x, y);
        m_rotation = rotation;
    }

    /**
     * Constructs a transform with the specified affine transformation matrix.
     *
     * @param matrix The affine transformation matrix.
     * @throws IllegalArgumentException if the affine transformation matrix is
     *                                  invalid.
     */
    public Transform2d(Matrix<N3, N3> matrix) {
        m_translation = new Translation2d(matrix.get(0, 2), matrix.get(1, 2));
        m_rotation = new Rotation2d(matrix.block(2, 2, 0, 0));
        if (matrix.get(2, 0) != 0.0 || matrix.get(2, 1) != 0.0 || matrix.get(2, 2) != 1.0) {
            throw new IllegalArgumentException("Affine transformation matrix is invalid");
        }
    }

    /** Constructs the identity transform -- maps an initial pose to itself. */
    public Transform2d() {
        m_translation = Translation2d.kZero;
        m_rotation = Rotation2d.kZero;
    }

    /**
     * Multiplies the transform by the scalar.
     *
     * @param scalar The scalar.
     * @return The scaled Transform2d.
     */
    public Transform2d times(double scalar) {
        return new Transform2d(m_translation.times(scalar), m_rotation.times(scalar));
    }

    /**
     * Divides the transform by the scalar.
     *
     * @param scalar The scalar.
     * @return The scaled Transform2d.
     */
    public Transform2d div(double scalar) {
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
    public Transform2d plus(Transform2d other) {
        return new Transform2d(Pose2d.kZero, Pose2d.kZero.transformBy(this).transformBy(other));
    }

    /**
     * Returns the translation component of the transformation.
     *
     * @return The translational component of the transform.
     */
    public Translation2d getTranslation() {
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
     * Returns an affine transformation matrix representation of this
     * transformation.
     *
     * @return An affine transformation matrix representation of this
     *         transformation.
     */
    public Matrix<N3, N3> toMatrix() {
        Vector<N2> vec = m_translation.toVector();
        Matrix<N2, N2> mat = m_rotation.toMatrix();
        return MatBuilder.fill(
                N3.instance,
                N3.instance,
                mat.get(0, 0),
                mat.get(0, 1),
                vec.get(0),
                mat.get(1, 0),
                mat.get(1, 1),
                vec.get(1),
                0.0,
                0.0,
                1.0);
    }

    /**
     * Returns the rotational component of the transformation.
     *
     * @return Reference to the rotational component of the transform.
     */
    public Rotation2d getRotation() {
        return m_rotation;
    }

    /**
     * Invert the transformation. This is useful for undoing a transformation.
     *
     * @return The inverted transformation.
     */
    public Transform2d inverse() {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        return new Transform2d(
                getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()),
                getRotation().unaryMinus());
    }

    @Override
    public String toString() {
        return String.format("Transform2d(%s, %s)", m_translation, m_rotation);
    }

    /**
     * Checks equality between this Transform2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Transform2d))
            return false;

        Transform2d other = (Transform2d) obj;
        return other.m_translation.equals(m_translation)
                && other.m_rotation.equals(m_rotation);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_translation, m_rotation);
    }
}