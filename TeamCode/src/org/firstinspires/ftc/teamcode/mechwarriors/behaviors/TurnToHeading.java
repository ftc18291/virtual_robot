package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class TurnToHeading extends Behavior {

    MechRobot robot;
    Telemetry telemetry;
    int desiredHeading;

    public TurnToHeading(Telemetry telemetry, String name, MechRobot robot, int heading) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = name;
        this.desiredHeading = -heading;
    }

    @Override
    public void start() {
        run();
    }

    @Override
    public void run() {
        double robotHeading = robot.getHeading();

        if (!almostEqual(robotHeading, desiredHeading, 0.5)) {
            telemetry.addData("robotHeading", robotHeading);
            double steeringCorrection = (robotHeading - desiredHeading) * 0.02;
            telemetry.addData("steeringCorrection", steeringCorrection);
            robot.mecanumDrive(0, 0, steeringCorrection);
        } else {
            robot.stop();
            isDone = true;
        }
    }

    public static boolean almostEqual(double a, double b, double difference) {
        return Math.abs(a - b) < difference;
    }
}
