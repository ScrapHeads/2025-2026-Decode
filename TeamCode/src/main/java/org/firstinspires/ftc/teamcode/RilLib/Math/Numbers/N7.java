package org.firstinspires.ftc.teamcode.RilLib.Math.Numbers;

import org.firstinspires.ftc.teamcode.RilLib.Math.Nat;
import org.firstinspires.ftc.teamcode.RilLib.Math.Num;

public class N7 extends Num implements Nat<N7> {
    /** Singleton instance of N7 */
    public static final N7 instance = new N7();

    /** Private constructor to enforce singleton pattern */
    private N7() {
    }

    /**
     * Get the number represented by this class
     * 
     * @return The number represented by this class, which is 7.
     */
    @Override
    public int getNum() {
        return 7;
    }

}
