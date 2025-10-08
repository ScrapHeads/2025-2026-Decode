package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N17 extends Num implements Nat<N17> {
    /** Singleton instance of N17 */
    public static final N17 instance = new N17();

    /** Private constructor to enforce singleton pattern */
    private N17() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 17.
     */
    @Override
    public int getNum() {
        return 17;
    }

}
