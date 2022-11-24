package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

@TeleOp(group = "MechWarriors")
public class PowerPlayOpMode extends OpMode {
    MechRobot robot;
    Claw claw;

    @Override
    public void init() {
        robot = new MechRobot(hardwareMap);
        claw = robot.getClaw();
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        robot.mecanumDrive(x, y, rx);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);

        telemetry.addData("Lift ticks: ", robot.getLift().getLiftTicks());

        if (gamepad1.dpad_up) {
            robot.getLift().liftArmUp();
            telemetry.addData("Lift", "Up");
        } else if (gamepad1.dpad_down) {
            robot.getLift().liftArmDown();
            telemetry.addData("Lift", "Down");
        } else {
            robot.getLift().liftArmStop();
            telemetry.addData("Lift", "Stop");
        }

        if (gamepad1.y) {
            claw.close();
            telemetry.addData("Claw", "Close");
        } else {
            claw.open();
            telemetry.addData("Claw", "Open");
        }
    }
}
