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

    double clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    int slidePos = 0;
    int armRotPos = 0;

    int highBucketPos = 2000;
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

        robot.closeClaw();

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

         if(gamepad2.right_bumper) {    // high bumper position
            robot.liftToPos(highBucketPos,0.7);
            // robot.rotateExtender(
         }

         if(gamepad2.left_bumper) { // low bumper position

         }

         // Intake
         if(gamepad2.dpad_down) {
             robot.lowerIntake();
         }

         if(gamepad2.dpad_up) {
             robot.raiseIntake();
         }

         if(gamepad2.a) {   // toggle intake
             if(this.intakeOn) {
                 robot.stopIntake();
                 intakeOn = false;
             }
             else {
                 float intensity = robot.getIntensity();
                 if(intensity < 0.0002) {
                     robot.runIntake();
                     intakeOn = true;
                 }
             }
         }


         if(gamepad2.dpad_left) {
             robot.resetToZero();   // todo
         }

         // Spec
         if(gamepad2.dpad_right) {
             robot.getSpecFromWall();   // todo
         }

         // Grasper
        if(gamepad2.x) {    //open grasper
            robot.openClaw();
        }
        if(gamepad2.b) {    // close grasper
            robot.closeClaw();
        }

        float trig = gamepad2.right_stick_y;
        if(trig > 0) {
            robot.rotateExtender((int) (robot.extender.getCurrentPosition() + trig), 0.6);
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