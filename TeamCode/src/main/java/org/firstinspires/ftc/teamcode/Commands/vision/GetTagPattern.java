package org.firstinspires.ftc.teamcode.Commands.vision;

import static org.firstinspires.ftc.teamcode.Constants.MIN_TAG_PIXEL_SIZE;
import static org.firstinspires.ftc.teamcode.Constants.PATTERN_TAG_IDS;
import static org.firstinspires.ftc.teamcode.Constants.patters;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class GetTagPattern extends CommandBase {

    private final Vision vision;
    private boolean setPatturn = false;

    public GetTagPattern (Vision vision) {
        this.vision = vision;

        addRequirements(vision);
    }

    @Override
    public void execute () {
        HuskyLens.Block[] blocks = vision.detectObject();

        for (HuskyLens.Block block : blocks) {
            boolean isRightTag = false;

            for (int id : PATTERN_TAG_IDS) {
                if (block.id == id) {
                    isRightTag = true;
                    break;
                }
            }

            if (!isRightTag) continue;
//            if (Math.min(block.width, block.height) < MIN_TAG_PIXEL_SIZE) continue;

            RobotState.getInstance().setPattern(patters.get(block.id));
            setPatturn = true;
        }
    }

    @Override
    public boolean isFinished () {
        return setPatturn;
    }

    @Override
    public void end (boolean interrupted) {
        if (RobotState.getInstance().getPattern() == null) {
            RobotState.getInstance().setPattern(new BallColor[] {
                    BallColor.PURPLE,
                    BallColor.PURPLE,
                    BallColor.GREEN
            }) ;
        }
    }

}
