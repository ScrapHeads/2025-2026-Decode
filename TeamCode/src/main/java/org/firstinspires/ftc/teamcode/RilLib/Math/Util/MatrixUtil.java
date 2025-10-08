package org.firstinspires.ftc.teamcode.RilLib.Math.Util;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

public final class MatrixUtil {
    private MatrixUtil() {}

    /**
     * Computes the matrix exponential of a square matrix
     *
     * @param src Row-major input array (rows x rows)
     * @param rows Number of rows (and columns)
     * @param dst Row-Major output array (must be length >= rows*rows)
     */
    public static void exp(double[] src, int rows, double[] dst) {
        if (src == null || dst == null) {
            throw new NullPointerException("src and dst must be non-null");
        }
        if (dst.length < rows * rows) {
            throw new IllegalArgumentException("dst too small");
        }

        // Build Apache Commons Matrix (row-major backing)
        double[][] data = new double[rows][rows];
        for (int r = 0; r < rows; ++r) {
            System.arraycopy(src, r * rows, data[r], 0, rows);
        }

        Array2DRowRealMatrix m = new Array2DRowRealMatrix(data);
        EigenDecomposition eig = new EigenDecomposition(m);

        // Reconstruct exp(A) = V * exp(D) * V^{-1}
        int n = rows;
        double[][] V = new double[n][n];
        double[][] expD = new double[n][n];
        for (int i = 0; i < n; ++i) {
            RealMatrix eigVec = eig.getV().getColumnMatrix(i);
            double[] vec = eig.getV().getColumn(i);
            for (int r = 0; r < n; ++r) {
                V[r][i] = vec[r];
            }
            for (int j = 0; j < n; ++j) expD[i][j] = 0.0;
            expD[i][i] = Math.exp(eig.getRealEigenvalue(i));
        }

        DMatrixRMaj Vmat = new DMatrixRMaj(V);
        DMatrixRMaj Vinv = new DMatrixRMaj(n, n);
        if (!CommonOps_DDRM.invert(Vmat, Vinv)) {
            throw new RuntimeException("Eigenvector matrix not invertible for exp");
        }

        DMatrixRMaj expDmat = new DMatrixRMaj(expD);
        DMatrixRMaj tmp = new DMatrixRMaj(n, n);
        DMatrixRMaj result = new DMatrixRMaj(n, n);

        CommonOps_DDRM.mult(Vmat, expDmat, tmp);
        CommonOps_DDRM.mult(tmp, Vinv, result);

        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                dst[r * n + c] = result.get(r, c);
            }
        }
    }

    /**
     * Computes the matrix power (A^exponent) for a square matrix.
     *
     * @param src Row-major input array (rows x rows)
     * @param rows Number of rows (and columns)
     * @param exponent The exponent to raise eigenvalues to
     * @param dst Row Major output array (must be length >= rows*rows)
     */
    public static void pow(double[] src, int rows, double exponent, double[] dst) {
        if (src == null || dst == null) {
            throw new NullPointerException("src and dst must be non-null");
        }
        if (dst.length < rows * rows) {
            throw new IllegalArgumentException("dst too small");
        }

        int n = rows;
        double[][] data = new double[n][n];
        for (int r = 0; r < n; ++r) {
            System.arraycopy(src, r * n, data[r], 0, n);
        }

        Array2DRowRealMatrix m = new Array2DRowRealMatrix(data);
        EigenDecomposition eig = new EigenDecomposition(m);

        double[][] V = new double[n][n];
        double[][] powD = new double[n][n];
        for (int i = 0; i < n; ++i) {
            double[] vec = eig.getV().getColumn(i);
            for (int r = 0; r < n; ++r) V[r][i] = vec[r];
            for (int j = 0; j < n; ++j) powD[i][j] = 0.0;
            powD[i][i] = Math.pow(eig.getRealEigenvalue(i), exponent);
        }

        DMatrixRMaj Vmat = new DMatrixRMaj(V);
        DMatrixRMaj Vinv = new DMatrixRMaj(n, n);
        if (!CommonOps_DDRM.invert(Vmat, Vinv)) {
            throw new RuntimeException("Eigenvector matrix not invertible for pow");
        }

        DMatrixRMaj powDmat = new DMatrixRMaj(powD);
        DMatrixRMaj tmp = new DMatrixRMaj(n, n);
        DMatrixRMaj result = new DMatrixRMaj(n, n);

        CommonOps_DDRM.mult(Vmat, powDmat, tmp);
        CommonOps_DDRM.mult(tmp, Vinv, result);

        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                dst[r * n + c] = result.get(r, c);
            }
        }
    }

    /**
     * Performs an in-place rank-one update (or downdate) of a triangular Cholesky factor.
     * The contents of {@code mat} are interpreted as a triangular Cholesky factor
     * (lower if {@code lowerTriangular} is true, otherwise upper). The triangular
     * factor is updated to be the factorization of (A + sigma * v * v^T), where A
     * is the original reconstructed matrix from the factor.
     *
     * @param mat Array holding the triangular factor in row-major form. It is mutated
     * @param rows Number of rows
     * @param vec Vector v used in the rank update
     * @param sigma Scaling for the rank update
     * @param lowerTriangular Whether the triangular factor is lower-triangular
     */
    public static void rankUpdate(
            double[] mat,
            int rows,
            double[] vec,
            double sigma,
            boolean lowerTriangular
    ) {
        if (mat == null || vec == null) throw new NullPointerException();
        if (mat.length < rows * rows) throw new IllegalArgumentException("mat too small");
        if (vec.length < rows) throw new IllegalArgumentException("vec too small");

        // Reconstruct full A from the triangular factor
        DMatrixRMaj T = new DMatrixRMaj(rows, rows);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < rows; ++c) {
                double vval = mat[r * rows + c];
                if (lowerTriangular) {
                    // only lower triangle stored
                    if (c <= r) T.set(r, c, vval);
                    else T.set(r, c, 0.0);
                } else {
                    // only upper triangle stored
                    if (c >= r) T.set(r, c, vval);
                    else T.set(r, c, 0.0);
                }
            }
        }

        DMatrixRMaj A = new DMatrixRMaj(rows, rows);
        if (lowerTriangular) {
            // A = L * L^T
            CommonOps_DDRM.mult(T, CommonOps_DDRM.transpose(T, null), A);
        } else {
            // A = U^T, * U
            DMatrixRMaj Tt = new DMatrixRMaj(rows, rows);
            CommonOps_DDRM.transpose(T, Tt);
            CommonOps_DDRM.mult(Tt, T, A);
        }

        // Add sigma * v * v^T
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < rows; ++j) {
                A.add(i, j, sigma * vec[i] * vec[j]);
            }
        }

        // Compute Cholesky of updated A using Apache Commons Math (returns lower L)
        double[][] Adata = new double[rows][rows];
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < rows; ++c) Adata[r][c] = A.get(r, c);
        }
        RealMatrix A_rm = new Array2DRowRealMatrix(Adata);
        CholeskyDecomposition chol = new CholeskyDecomposition(A_rm);
        RealMatrix Lrm = chol.getL(); // lower-triangular L

        // Write updated triangular factor back into mat in the same triangular layout
        if (lowerTriangular) {
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c <= r; ++c) {
                    mat[r * rows + c] = Lrm.getEntry(r, c);
                }
                for (int c = r + 1; c < rows; ++c) {
                    mat[r * rows + c] = 0.0;
                }
            }
        } else {
            // store upper triangular U = L^T
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < r; ++c) {
                    mat[r * rows + c] = 0.0;
                }
                for (int c = r; c < rows; ++c) {
                    mat[r * rows + c] = Lrm.getEntry(c, r);
                }
            }
        }
    }

    /**
     * Solves A * X = B for X using a pseudo-inverse (handles rectangular and
     * rank-deficient cases). A is Arows x Acols. B is Brows x Bcols. Result X is
     * (Acols x Bcols) and written into dst in row-major order.
     */
    public static void solveFullPivHouseholderQr(
            double[] A,
            int Arows,
            int Acols,
            double[] B,
            int Brows,
            int Bcols,
            double[] dst
    ) {
        if (A == null || B == null || dst == null) throw new NullPointerException();
        if (A.length < Arows * Acols) throw new IllegalArgumentException("A too small");
        if (B.length < Brows * Bcols) throw new IllegalArgumentException("B too small");
        if (dst.length < Acols * Bcols) throw new IllegalArgumentException("dst too small");

        DMatrixRMaj Am = new DMatrixRMaj(Arows, Acols);
        DMatrixRMaj Bm = new DMatrixRMaj(Brows, Bcols);
        for (int r = 0; r < Arows; ++r) for (int c = 0; c < Acols; ++c) Am.set(r, c, A[r * Acols + c]);
        for (int r = 0; r < Brows; ++r) for (int c = 0; c < Bcols; ++c) Bm.set(r, c, B[r * Bcols + c]);

        SimpleMatrix As = SimpleMatrix.wrap(Am);
        SimpleMatrix Bs = SimpleMatrix.wrap(Bm);

        // Use pseudo-inverse to handle full-pivot QR behavior in general cases
        SimpleMatrix pinvA = As.pseudoInverse();
        SimpleMatrix X = pinvA.mult(Bs);

        for (int r = 0; r < Acols; ++r) {
            for (int c = 0; c < Bcols; ++c) {
                dst[r * Bcols + c] = X.get(r, c);
            }
        }
    }
}
