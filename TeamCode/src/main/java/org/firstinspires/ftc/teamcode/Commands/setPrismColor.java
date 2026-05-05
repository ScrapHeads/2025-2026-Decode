package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.prism.Color;
import org.firstinspires.ftc.teamcode.subsystems.PrismLights;

public class setPrismColor extends CommandBase {

    PrismLights prism;
    Color color;

    public setPrismColor (PrismLights prism, Color color) {
        this.prism = prism;
        this.color = color;
    }

    @Override
    public void initialize () {
        prism.setColor(color);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
