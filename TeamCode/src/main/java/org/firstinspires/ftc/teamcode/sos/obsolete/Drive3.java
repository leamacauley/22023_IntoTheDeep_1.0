package org.firstinspires.ftc.teamcode.sos.obsolete;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.sos.BaseRobot;


//@Config
@Disabled
@TeleOp (name = "DRIVE CODE V2: ROBOT CENTRIC")

public class Drive3 extends OpMode {

    /* Declare OpMode members. */

    BaseRobot robot = new BaseRobot();

    double clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    int slidePos = 0;
    int armRotPos = 0;
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
        /**

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        /**
         * UPDATED CONTROLS
         *      REFER TO GOOGLE DRIVE FOR ASSIGNMENTS


        // GAMEPAD 1 ***********************************************************
        robot.closeClaw();

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

        // Slider
        if(slidePos > 3210) {  // safety code
            slidePos = 2210;
        }
        if(slidePos < 0) {
            slidePos = 0;
        }

        robot.leftLift.setTargetPosition(slidePos);
        robot.rightLift.setTargetPosition(slidePos);
        telemetry.addData("Say", "SlidePos: " + slidePos);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLift.setPower(0.8);
        robot.rightLift.setPower(0.8);


        // Slider
        double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;
        if(slidePower > 0.1) {
            slidePos += (gamepad2.right_trigger * 100);
        }
        if(slidePower < -0.1) {
            slidePos -= (gamepad2.left_trigger * 100);
        }

        if(slidePos > 3210) {  // safety code
            slidePos = 2210;
        }
        if(slidePos < 0) {
            slidePos = 0;
        }

        // Intake Arm
        /**
        if(gamepad2.dpad_down) {    //INTAKE POS
            robot.shoulder.setPosition(0.3);
            robot.rotateArm(1000, 0.7);
        }

        if(gamepad2.dpad_up) {    // ZERO POS
            robot.rotateArm(0,0.7);
            robot.shoulder.setPosition(0.5);
        }
        if(gamepad2.x) {
            robot.shoulder.setPosition(0.3);
        }


        if(gamepad2.dpad_down) {
            robot.openClaw();
        }
        if(gamepad2.dpad_up) {
            robot.closeClaw();
        }

        // Intake
        if(gamepad2.dpad_right) {
            if(this.intakeOn) { // toggle
                robot.runIntake();
                intakeOn = false;
                return;
            }
            else {
                robot.stopIntake();
                intakeOn = true;
                return;
            }

        }


        // Scoring Controls
        if(gamepad2.left_bumper) {  // raise slider and raise pivot to high pos
            robot.liftToPos(1200,0.7);
            robot.hinge.setPosition(0.1);
        }
        if(gamepad2.right_bumper) { // low pos
           robot.hinge.setPosition(0.1);
        }
        if(gamepad2.a) {
            robot.liftToPos(0,0.8);
            robot.waitForTick(500);
            robot.openClaw();
        }

        // Spec Grab controls
        if(gamepad2.y) {
            robot.hinge.setPosition(0.2);
        }

        if(gamepad2.b) {
            robot.hinge.setPosition(0.4);
        }

        // Color sensor
        if(robot.sensorBlocked()) {
            robot.stopIntake();
        }

        robot.wrist.setPosition(0.65);

        */
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}