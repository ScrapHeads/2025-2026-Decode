package org.firstinspires.ftc.teamcode.Commands.vision;

import static org.firstinspires.ftc.teamcode.Constants.PATTERN_TAG_IDS;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.patters;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class GetTagPattern extends CommandBase {

    private final Vision vision;
    private boolean setPattern = false;

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
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Right Pattern", patters.get(block.id));
            dashboard.sendTelemetryPacket(packet);
            RobotState.getInstance().setPattern(patters.get(block.id));
            setPattern = true;
        }
    }

    @Override
    public boolean isFinished () {
        return setPattern;
    }

    @Override
    public void end (boolean interrupted) {
        if (RobotState.getInstance().getPattern() == null && !setPattern) {
            RobotState.getInstance().setPattern(new BallColor[] {
                    BallColor.PURPLE,
                    BallColor.PURPLE,
                    BallColor.GREEN
            }) ;
        }
    }

}
