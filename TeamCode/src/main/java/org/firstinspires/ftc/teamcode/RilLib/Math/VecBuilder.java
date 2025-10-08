// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.RilLib.Math;

import java.util.Objects;

import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N1;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N2;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N4;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N5;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N6;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N7;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N8;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N9;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N10;
import org.ejml.simple.SimpleMatrix;

/** A class for constructing vectors (Nx1 matrices). */
public final class VecBuilder {
    private VecBuilder() {
        throw new UnsupportedOperationException("this is a utility class!");
    }

    private static <N extends Num> Vector<N> fillVec(Nat<N> rows, double... data) {
        if (Objects.requireNonNull(data).length != rows.getNum()) {
            throw new IllegalArgumentException(
                    "Invalid vector data provided. Wanted "
                            + rows.getNum()
                            + " vector, but got "
                            + data.length
                            + " elements");
        }
        return new Vector<>(new SimpleMatrix(data));
    }

    /**
     * Returns a 1x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @return 1x1 vector
     */
    public static Vector<N1> fill(double n1) {
        return fillVec(N1.instance, n1);
    }

    /**
     * Returns a 2x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @return 2x1 vector
     */
    public static Vector<N2> fill(double n1, double n2) {
        return fillVec(N2.instance, n1, n2);
    }

    /**
     * Returns a 3x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @param n3 the third element.
     * @return 3x1 vector
     */
    public static Vector<N3> fill(double n1, double n2, double n3) {
        return fillVec(N3.instance, n1, n2, n3);
    }

    /**
     * Returns a 4x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @param n3 the third element.
     * @param n4 the fourth element.
     * @return 4x1 vector
     */
    public static Vector<N4> fill(double n1, double n2, double n3, double n4) {
        return fillVec(N4.instance, n1, n2, n3, n4);
    }

    /**
     * Returns a 5x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @param n3 the third element.
     * @param n4 the fourth element.
     * @param n5 the fifth element.
     * @return 5x1 vector
     */
    public static Vector<N5> fill(double n1, double n2, double n3, double n4, double n5) {
        return fillVec(N5.instance, n1, n2, n3, n4, n5);
    }

    /**
     * Returns a 6x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @param n3 the third element.
     * @param n4 the fourth element.
     * @param n5 the fifth element.
     * @param n6 the sixth element.
     * @return 6x1 vector
     */
    public static Vector<N6> fill(double n1, double n2, double n3, double n4, double n5, double n6) {
        return fillVec(N6.instance, n1, n2, n3, n4, n5, n6);
    }

    /**
     * Returns a 7x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @param n3 the third element.
     * @param n4 the fourth element.
     * @param n5 the fifth element.
     * @param n6 the sixth element.
     * @param n7 the seventh element.
     * @return 7x1 vector
     */
    public static Vector<N7> fill(
            double n1, double n2, double n3, double n4, double n5, double n6, double n7) {
        return fillVec(N7.instance, n1, n2, n3, n4, n5, n6, n7);
    }

    /**
     * Returns a 8x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @param n3 the third element.
     * @param n4 the fourth element.
     * @param n5 the fifth element.
     * @param n6 the sixth element.
     * @param n7 the seventh element.
     * @param n8 the eighth element.
     * @return 8x1 vector
     */
    public static Vector<N8> fill(
            double n1, double n2, double n3, double n4, double n5, double n6, double n7, double n8) {
        return fillVec(N8.instance, n1, n2, n3, n4, n5, n6, n7, n8);
    }

    /**
     * Returns a 9x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @param n3 the third element.
     * @param n4 the fourth element.
     * @param n5 the fifth element.
     * @param n6 the sixth element.
     * @param n7 the seventh element.
     * @param n8 the eighth element.
     * @param n9 the ninth element.
     * @return 9x1 vector
     */
    public static Vector<N9> fill(
            double n1,
            double n2,
            double n3,
            double n4,
            double n5,
            double n6,
            double n7,
            double n8,
            double n9) {
        return fillVec(N9.instance, n1, n2, n3, n4, n5, n6, n7, n8, n9);
    }

    /**
     * Returns a 10x1 vector containing the given elements.
     *
     * @param n1 the first element.
     * @param n2 the second element.
     * @param n3 the third element.
     * @param n4 the fourth element.
     * @param n5 the fifth element.
     * @param n6 the sixth element.
     * @param n7 the seventh element.
     * @param n8 the eighth element.
     * @param n9 the ninth element.
     * @param n10 the tenth element.
     * @return 10x1 vector
     */
    public static Vector<N10> fill(
            double n1,
            double n2,
            double n3,
            double n4,
            double n5,
            double n6,
            double n7,
            double n8,
            double n9,
            double n10) {
        return fillVec(N10.instance, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10);
    }
}