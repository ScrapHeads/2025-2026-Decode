package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N5 extends Num implements Nat<N5> {
    /** Singleton instance of N5 */
    public static final N5 instance = new N5();

    /** Private constructor to enforce singleton pattern */
    private N5() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 5.
     */
    @Override
    public int getNum() {
        return 5;
    }

}
