package org.firstinspires.ftc.teamcode.sos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


//@Config
@TeleOp (name = "DRIVE CODE V3: ROBOT CENTRIC")

public class Drive4 extends OpMode {

    /* Declare OpMode members. */

    BaseRobot robot = new BaseRobot();


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
        robot.init(hardwareMap);

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

         // Retrieve the IMU from the hardware map
         IMU imu = hardwareMap.get(IMU.class, "imu");
         // Adjust the orientation parameters to match your robot
         IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
         RevHubOrientationOnRobot.LogoFacingDirection.UP,
         RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
         // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
         imu.initialize(parameters);


         // GAMEPAD 1 ***********************************************************


         // SLOW MODE
         if(gamepad1.right_trigger >= 0.1) {
         SPEED_CONTROL = 0.3;
         }
         else{
         SPEED_CONTROL = 0.8;
         }

         // Movement
         double leftX;
         double leftY;
         double rightX;

         leftX = -gamepad1.left_stick_x;
         leftY = -gamepad1.left_stick_y ;
         rightX = -gamepad1.right_stick_x;

         double leftRearPower = (leftY + leftX - rightX);
         double leftFrontPower = (leftY - leftX - rightX);
         double rightRearPower = (leftY - leftX + rightX);
         double rightFrontPower = (leftY + leftX + rightX);

         robot.leftFront.setPower(leftFrontPower * SPEED_CONTROL);
         robot.leftRear.setPower(leftRearPower * SPEED_CONTROL);
         robot.rightFront.setPower(rightFrontPower * SPEED_CONTROL);
         robot.rightRear.setPower(rightRearPower * SPEED_CONTROL);


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

         //double extendPos = gamepad1.right_trigger - gamepad1.left_trigger;
         //robot.manualExtenderMove(extendPos * 10);

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
             robot.leftLift.setTargetPosition(1000);
             robot.rightLift.setTargetPosition(1000);
             //telemetry.addData("Say", "SlidePos: " + slidePos);
             robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             robot.leftLift.setPower(0.8);
             robot.rightLift.setPower(0.8);

             robot.getSpecFromWall();
         }

         // Grasper
        if(gamepad2.x) {    //open grasper
            robot.openClaw();
        }
        if(gamepad2.b) {    // close grasper
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