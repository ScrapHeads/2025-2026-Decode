package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N10 extends Num implements Nat<N10> {
    /** Singleton instance of N10 */
    public static final N10 instance = new N10();

    /** Private constructor to enforce singleton pattern */
    private N10() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 10.
     */
    @Override
    public int getNum() {
        return 10;
    }

}
