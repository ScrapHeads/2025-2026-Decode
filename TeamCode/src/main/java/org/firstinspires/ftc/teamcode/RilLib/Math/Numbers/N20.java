package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N20 extends Num implements Nat<N20> {
    /** Singleton instance of N20 */
    public static final N20 instance = new N20();

    /** Private constructor to enforce singleton pattern */
    private N20() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 20.
     */
    @Override
    public int getNum() {
        return 20;
    }

}
