package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N16 extends Num implements Nat<N16> {
    /** Singleton instance of N16 */
    public static final N16 instance = new N16();

    /** Private constructor to enforce singleton pattern */
    private N16() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 16.
     */
    @Override
    public int getNum() {
        return 16;
    }

}
