package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.DriveHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.TurnToHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "MechWarriors")
public class PowerPlayAutoOpMode extends OpMode {

    MechRobot robot;

    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LOWER;

    List<Behavior> behaviors = new ArrayList<Behavior>();
    int state = 0;

    @Override
    public void init() {
        robot = new MechRobot(hardwareMap);
        telemetry.addLine("Init done");
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) {
            startingLocation = StartingLocation.UPPER;
        } else if (gamepad1.dpad_down) {
            startingLocation = StartingLocation.LOWER;
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

        if (startingLocation == StartingLocation.LOWER && allianceColor == AllianceColor.BLUE) {
            behaviors.add(new TurnToHeading(telemetry, "turn right to 90", robot, 90));
            behaviors.add(new DriveHeading(telemetry, "drive forward 1", robot, 90, 2000));
        } else if (startingLocation == StartingLocation.UPPER && allianceColor == AllianceColor.BLUE) {
            behaviors.add(new TurnToHeading(telemetry, "turn left to -90", robot, -90));
            behaviors.add(new DriveHeading(telemetry, "drive forward 2", robot, 0, 2000));
        } else if (startingLocation == StartingLocation.LOWER && allianceColor == AllianceColor.RED) {
            behaviors.add(new TurnToHeading(telemetry, "turn left to -180", robot, -180));
        } else if (startingLocation == StartingLocation.UPPER && allianceColor == AllianceColor.RED) {
            behaviors.add(new TurnToHeading(telemetry, "turn right to 180", robot, 180));
        }

        behaviors.get(0).start();
    }

    @Override
    public void loop() {
        telemetry.addLine("Running");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);

        if (state < behaviors.size()) {
            if (!behaviors.get(state).isDone()) {
                //System.out.println("state " + state + " is running");
                telemetry.addData("Running behavior:", behaviors.get(state).getName());
                behaviors.get(state).run();
            } else {
                System.out.println("state " + state + " is done");
                state++;
                if (state < behaviors.size()) {
                    System.out.println("starting state " + state);
                    behaviors.get(state).start();
                }
            }
        } else {
            this.stop();
        }

    }
}
