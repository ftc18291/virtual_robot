package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.*;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "MechWarriors")
public class PowerPlayAutoOpMode extends OpMode {

    MechRobot robot;

    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.FRONT;

    List<Behavior> behaviors = new ArrayList<Behavior>();
    int state = 0;

    @Override
    public void init() {
        robot = new MechRobot(hardwareMap);
        telemetry.addLine("Init done");
    }

    @Override
    public void init_loop() {
        if (gamepad1.y) {
            startingLocation = StartingLocation.BACK;
        } else if (gamepad1.a) {
            startingLocation = StartingLocation.FRONT;
        }

        if (gamepad1.x) {
            allianceColor = AllianceColor.BLUE;
        } else if (gamepad1.b) {
            allianceColor = AllianceColor.RED;
        }

        telemetry.addLine("Select Location and Alliance Color");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);
    }

    @Override
    public void start() {

        if (startingLocation == StartingLocation.FRONT && allianceColor == AllianceColor.BLUE) {
            behaviors.add(new RaiseLift(telemetry, "raise lift to medium", robot, JunctionType.MEDIUM));
            behaviors.add(new OpenClaw(telemetry, "raise lift to medium", robot.getClaw()));
            behaviors.add(new TurnToHeading(telemetry, "turn right to 90", robot, 90));
            behaviors.add(new DriveHeading(telemetry, "drive forward 1", robot, 90, 2000, 0.5));
        } else if (startingLocation == StartingLocation.BACK && allianceColor == AllianceColor.BLUE) {
            behaviors.add(new TurnToHeading(telemetry, "turn left to -90", robot, -90));
            behaviors.add(new DriveHeading(telemetry, "drive forward 2", robot, -90, 3000, 1.0));
        } else if (startingLocation == StartingLocation.FRONT && allianceColor == AllianceColor.RED) {
            behaviors.add(new TurnToHeading(telemetry, "turn left to -180", robot, -180));
        } else if (startingLocation == StartingLocation.BACK && allianceColor == AllianceColor.RED) {
            behaviors.add(new TurnToHeading(telemetry, "turn right to 180", robot, 225));
        }

        behaviors.get(0).start();
    }

    @Override
    public void loop() {
        telemetry.addLine("Running program...");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);

        if (state < behaviors.size()) {
            if (!behaviors.get(state).isDone()) {
                telemetry.addData("Running behavior:", behaviors.get(state).getName());
                behaviors.get(state).run();
            } else {
                state++;
                if (state < behaviors.size()) {
                    behaviors.get(state).start();
                }
            }
        } else {
            telemetry.addLine("Program done");
            this.stop();
        }
    }
}
