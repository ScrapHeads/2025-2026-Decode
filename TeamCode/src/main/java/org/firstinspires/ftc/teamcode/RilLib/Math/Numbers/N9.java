package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N9 extends Num implements Nat<N9> {
    /** Singleton instance of N9 */
    public static final N9 instance = new N9();

    /** Private constructor to enforce singleton pattern */
    private N9() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 9.
     */
    @Override
    public int getNum() {
        return 9;
    }

}
