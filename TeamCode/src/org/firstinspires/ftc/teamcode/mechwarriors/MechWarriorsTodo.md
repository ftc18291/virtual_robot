1. Open hardware/MechRobot.java
    1. Implement TODO 1 in calculateDriveTicks
    2. Implement TODO 2 in getDriveTicks
    3. Implement TODOs 3 & 4 in liftArmUp
    4. Implement TODOs 5 & 6 in liftArmDown

2. Look at Claw interface
    1. Create a new class call EthanClaw that implements the Claw interface
        1. In the constructor, setup the servos
        
                leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
                leftClawServo.setDirection(Servo.Direction.REVERSE);
                rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");
                leftClawServo.setPosition(0);
                rightClawServo.setPosition(0);
        2. In the open method
        
                leftClawServo.setPosition(0);
                rightClawServo.setPosition(0);
        3. In the close method
        
                leftClawServo.setPosition(1.0);
                rightClawServo.setPosition(1.0);
    2. In MechRobot.java, implement TODO 7
    3. In the PowerPlayOpMode.java, move the claw
    
            if (gamepad1.y) {
                claw.close();
                telemetry.addData("Claw", "Close");
            } else {
                claw.open();
                telemetry.addData("Claw", "Open");
            }
