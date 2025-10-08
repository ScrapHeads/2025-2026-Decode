package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N6 extends Num implements Nat<N6> {
    /** Singleton instance of N6 */
    public static final N6 instance = new N6();

    /** Private constructor to enforce singleton pattern */
    private N6() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 6.
     */
    @Override
    public int getNum() {
        return 6;
    }

}
