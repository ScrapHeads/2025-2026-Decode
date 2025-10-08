package org.firstinspires.ftc.teamcode.RilLib.Math.Util;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose3d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Quaternion;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Translation3d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.RilLib.Math.Vector;

public class Pose3dUtil {
    private Pose3dUtil() {}

    public static double vecNorm(double[] v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    public static double[][] omegaFromVector(double[] r) {
        return new double[][] {
                {0.0, -r[2], r[1]},
                {r[2], 0.0, -r[0]},
                {-r[1], r[0], 0.0}
        };
    }

    public static double[] matVecMul(double[][] m, double[] v) {
        double[] out = new double[3];
        for (int i = 0; i < 3; ++i) {
            out[i] = m[i][0] * v[0] + m[i][1] * v[1] +m[i][2] * v[2];
        }
        return out;
    }

    public static double[][] matMul(double[][] a, double[][] b) {
        double[][] r = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    sum += a[i][k] * b[k][j];
                }
                r[i][j] = sum;
            }
        }
        return r;
    }
}
