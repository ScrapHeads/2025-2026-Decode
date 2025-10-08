package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N8 extends Num implements Nat<N8> {
    /** Singleton instance of N8 */
    public static final N8 instance = new N8();

    /** Private constructor to enforce singleton pattern */
    private N8() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 8.
     */
    @Override
    public int getNum() {
        return 8;
    }

}
