package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PowerPlayOpMode", group = "MechWarriors")
public class PowerPlayOpMode extends OpMode {
    MechRobot rb;
    Claw claw;

    @Override
    public void init() {
        rb = new MechRobot(hardwareMap);
        claw = rb.getClaw();
        //double ticks = rb.calculateLiftTicks(5);
        //telemetry.addData("ticks", ticks);
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        rb.mecanumDrive(x, y, rx);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);

        if (gamepad1.dpad_up) {
            rb.liftArmUp();
            telemetry.addData("Lift", "Up");
        } else if (gamepad1.dpad_down) {
            rb.liftArmDown();
            telemetry.addData("Lift", "Down");
        } else {
            rb.liftArmStop();
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
