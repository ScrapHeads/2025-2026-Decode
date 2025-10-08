package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N0 extends Num implements Nat<N0> {
    /** Singleton instance of N0 */
    public static final N0 instance = new N0();

    /** Private constructor to enforce singleton pattern */
    private N0() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 0.
     */
    @Override
    public int getNum() {
        return 0;
    }

}
