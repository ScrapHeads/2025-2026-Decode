package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N13 extends Num implements Nat<N13> {
    /** Singleton instance of N13 */
    public static final N13 instance = new N13();

    /** Private constructor to enforce singleton pattern */
    private N13() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 13.
     */
    @Override
    public int getNum() {
        return 13;
    }

}
