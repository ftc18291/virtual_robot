package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class LowerLift extends Behavior {
    MechRobot robot;
    JunctionType junctionType;

    public LowerLift(Telemetry telemetry, MechRobot robot, JunctionType junctionType) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.junctionType = junctionType;
        this.name = "Raise Lift = [Junction Type: " + junctionType + "]";
    }

    @Override
    public void start() {
        run();
    }

    @Override
    public void run() {
        if (robot.getLift().getLiftTicks() >= junctionType.getTicks()) {
            robot.getLift().liftArmDown();
        } else {
            robot.getLift().liftArmStop();
            this.isDone = true;
        }
    }
}

