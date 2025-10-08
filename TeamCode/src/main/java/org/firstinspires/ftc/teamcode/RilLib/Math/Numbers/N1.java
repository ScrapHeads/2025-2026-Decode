package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N1 extends Num implements Nat<N1> {
    /** Singleton instance of N1 */
    public static final N1 instance = new N1();

    /** Private constructor to enforce singleton pattern */
    private N1() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 1.
     */
    @Override
    public int getNum() {
        return 1;
    }

}
