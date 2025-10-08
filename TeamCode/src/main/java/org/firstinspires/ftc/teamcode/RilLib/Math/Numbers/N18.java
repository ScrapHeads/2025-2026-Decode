package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N18 extends Num implements Nat<N18> {
    /** Singleton instance of N18 */
    public static final N18 instance = new N18();

    /** Private constructor to enforce singleton pattern */
    private N18() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 18.
     */
    @Override
    public int getNum() {
        return 18;
    }

}
