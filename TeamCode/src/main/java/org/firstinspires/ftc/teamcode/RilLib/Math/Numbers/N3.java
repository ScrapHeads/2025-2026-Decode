package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N3 extends Num implements Nat<N3> {
    /** Singleton instance of N3 */
    public static final N3 instance = new N3();

    /** Private constructor to enforce singleton pattern */
    private N3() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 3.
     */
    @Override
    public int getNum() {
        return 3;
    }

}
