package org.firstinspires.ftc.teamcode.sos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


//@Config
@TeleOp (name = "FIELD CENTRIC DRIVE CODE")

public class Drive5 extends OpMode {

    /* Declare OpMode members. */

    BaseRobot robot = new BaseRobot();

    IMU imu;

    int sliderPos;
    int armRotPos = 0;

    int highBucketPos = 3100;
    private double SPEED_CONTROL = 0.8;

    boolean intakeOn = false;
    boolean grasping = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        // Initialize hardware
        robot.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armRotPos = robot.arm.getCurrentPosition();

        robot.openClaw();

        sliderPos = robot.rightLift.getCurrentPosition();
        //robot.pivot.setPosition(0.0);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = - gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if(gamepad1.right_trigger > 0.01) {
                SPEED_CONTROL = 0.2;
            }
            else {
                SPEED_CONTROL = 0.8;
            }
            robot.leftFront.setPower(frontLeftPower * SPEED_CONTROL);
            robot.leftRear.setPower(backLeftPower * SPEED_CONTROL);
            robot.rightFront.setPower(frontRightPower * SPEED_CONTROL);
            robot.rightRear.setPower(backRightPower * SPEED_CONTROL);


        // GAMEPAD 2 ***********************************************************

        // Right Arm (Slider) Code
        if (gamepad2.right_trigger > 0.1) { // Arm Up
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setPower(0.8);
            robot.leftLift.setPower(0.8);
        }
        else if (gamepad2.left_trigger > 0.1) { // Arm Down
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setPower(-0.8);
            robot.leftLift.setPower(-0.8);
        }
        else {
            sliderPos = robot.rightLift.getCurrentPosition();
            robot.rightLift.setTargetPosition(sliderPos);
            robot.leftLift.setTargetPosition(sliderPos);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightLift.setPower(0.8);
            robot.leftLift.setPower(0.8);
        }

        if(gamepad2.right_stick_y > 0.001 || gamepad2.right_stick_y < -0.001) {
            double extendPos = gamepad2.right_stick_y;
            robot.manualExtenderMove(extendPos * 10);
        }

        if(gamepad1.y) {
            robot.rotateExtender(2500,0.7);
        }

        if(gamepad2.right_bumper) {    // high bumper position
            robot.leftLift.setTargetPosition(3100);
            robot.rightLift.setTargetPosition(3100);
            //telemetry.addData("Say", "SlidePos: " + slidePos);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);

            robot.waitForTick(1400);
            robot.setScoringPos();
            robot.waitForTick(2000);
            robot.setScoringPos();
        }

        if(gamepad2.left_bumper) { // low bumper position
            robot.leftLift.setTargetPosition(1000);
            robot.rightLift.setTargetPosition(1000);
            //telemetry.addData("Say", "SlidePos: " + slidePos);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);

            robot.waitForTick(1400);
            robot.setScoringPos();
            robot.waitForTick(1000);

            robot.leftLift.setTargetPosition(0);
            robot.rightLift.setTargetPosition(0);
            //telemetry.addData("Say", "SlidePos: " + slidePos);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);
        }

        // Intake
        if(gamepad2.dpad_down) {
            robot.lowerIntake();
        }

        if(gamepad2.dpad_up) {
            robot.raiseIntake();
        }

        float intensity = robot.getIntensity();
        telemetry.addData("Color: ", intensity);
        telemetry.update();

        boolean colorDetected = (intensity > 0.0001);

        if (gamepad2.a && !colorDetected) {
            // Manual control: Run intake if button A is pressed and no color is detected
            robot.runIntake();
        } else if (gamepad2.ps) {
            // Manual override: Run intake if the PS button is pressed, regardless of color detection
            robot.runIntake();
        } else if (colorDetected) {
            // Stop intake if color is detected (unless overridden by manual control)
            robot.stopIntake();
        } else {
            // Stop intake by default if none of the above conditions are true
            robot.stopIntake();
        }

        if(gamepad2.dpad_left) {
            robot.leftLift.setTargetPosition(0);
            robot.rightLift.setTargetPosition(0);
            //telemetry.addData("Say", "SlidePos: " + slidePos);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);


            robot.resetToZero();

            robot.leftLift.setTargetPosition(0);
            robot.rightLift.setTargetPosition(0);
            //telemetry.addData("Say", "SlidePos: " + slidePos);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);

        }

        // Spec
        if(gamepad2.dpad_right) {
            robot.putOnBar();
        }

        // Grasper
        if(gamepad2.x) {    //open grasper
            robot.openClaw();
            robot.openSpecGrasper();
        }
        if(gamepad2.b) {    // close grasper
            robot.closeSpecGrasper();
            robot.closeClaw();
        }

        if(gamepad2.start) {
            robot.transfer();
        }


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}