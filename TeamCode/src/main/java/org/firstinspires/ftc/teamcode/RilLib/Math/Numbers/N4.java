package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N4 extends Num implements Nat<N4> {
    /** Singleton instance of N4 */
    public static final N4 instance = new N4();

    /** Private constructor to enforce singleton pattern */
    private N4() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 4.
     */
    @Override
    public int getNum() {
        return 4;
    }

}
