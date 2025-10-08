package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N15 extends Num implements Nat<N15> {
    /** Singleton instance of N15 */
    public static final N15 instance = new N15();

    /** Private constructor to enforce singleton pattern */
    private N15() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 15.
     */
    @Override
    public int getNum() {
        return 15;
    }

}
