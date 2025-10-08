package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N11 extends Num implements Nat<N11> {
    /** Singleton instance of N11 */
    public static final N11 instance = new N11();

    /** Private constructor to enforce singleton pattern */
    private N11() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 11.
     */
    @Override
    public int getNum() {
        return 11;
    }

}
