package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class DriveHeading extends Behavior {
    MechRobot robot;
    Telemetry telemetry;
    int heading;
    int distance;

    public DriveHeading(Telemetry telemetry, String name, MechRobot robot, int heading, int distance) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = name;
        this.heading = -heading;
        this.distance = distance;
    }

    @Override
    public void start() {
        robot.resetMotorTicks();
        run();
    }

    @Override
    public void run() {
        if (robot.getDriveTicks() < distance) {
            double robotHeading = robot.getHeading();
            telemetry.addData("robotHeading", robotHeading);
            double steeringCorrection = (robotHeading - heading) * 0.02;
            telemetry.addData("steeringCorrection", steeringCorrection);
            robot.mecanumDrive(0, 0.5, steeringCorrection);
        } else {
            robot.stop();
            isDone = true;
        }
    }
}
