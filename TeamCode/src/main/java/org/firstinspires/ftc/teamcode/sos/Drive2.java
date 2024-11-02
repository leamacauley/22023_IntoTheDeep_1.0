package org.firstinspires.ftc.teamcode.sos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


//@Config
@TeleOp (name = "DRIVE CODE: ROBOT CENTRIC")

public class Drive2 extends OpMode {

    /* Declare OpMode members. */

    BaseRobot robot = new BaseRobot();


    double clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    int extendPos = 0;
    private double SPEED_CONTROL = 0.8;

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

        //robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //extendPos = robot.extender.getCurrentPosition();

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

        // Movement
        double leftX;
        double leftY;
        double rightX;

        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y ;
        rightX = -gamepad1.right_stick_x;

        double leftRearPower = (leftY + leftX - rightX);
        double leftFrontPower = (leftY - leftX - rightX);
        double rightRearPower = (leftY - leftX + rightX);
        double rightFrontPower = (leftY + leftX + rightX);

        robot.leftFront.setPower(leftFrontPower * SPEED_CONTROL);
        robot.leftRear.setPower(leftRearPower * SPEED_CONTROL);
        robot.rightFront.setPower(rightFrontPower * SPEED_CONTROL);
        robot.rightRear.setPower(rightRearPower * SPEED_CONTROL);

        int currentSlidePos = robot.leftLift.getCurrentPosition();
        if(gamepad1.dpad_up) {
            robot.liftToPos(currentSlidePos+=100,0.8);
        }

        if(gamepad1.dpad_left) {
            robot.liftToPos(1500,0.8);
        }
        if(gamepad1.dpad_down) {
            robot.liftToPos(0, 0.8);
        }
        if(gamepad1.dpad_right) {
            robot.liftToPos(3000,0.8);
        }

        if(gamepad1.right_trigger >= 0.1) {
            SPEED_CONTROL = 0.3;
        }
        else{
            SPEED_CONTROL = 0.8;
        }


        if(gamepad2.right_bumper) {
            robot.runIntake();
        }
        if(gamepad2.left_bumper) {
            robot.stopIntake();
        }
        if(gamepad2.x){
            robot.runIntake();
        }

        if(gamepad2.y) {
            robot.rotateShoulder(0.0);
        }
        if(gamepad2.b) {
            robot.rotateShoulder(0.5);
        }
        if(gamepad2.a) {    // bring down
            robot.rotateShoulder(0.85);
        }

        if(gamepad2.dpad_up) {
            robot.rotateWrist(0.4);
        }
        if(gamepad2.dpad_down) {
            robot.rotateWrist(0.25);
        }


        // controlling extender
        double extendPower = gamepad2.right_trigger - gamepad2.left_trigger;
        if(extendPower > 0.1) {
            extendPos+= (gamepad2.right_trigger * 100);
        }
        if(extendPower < -0.1) {
            extendPos-= (gamepad2.left_trigger * 100);
        }

        if(extendPos > 2210) {  // safety code
            extendPos = 2210;
        }
        //robot.extender.setTargetPosition(extendPos);
        telemetry.addData("Say", "ExtendPos: " + extendPos);
        //robot.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.extender.setPower(0.8);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}