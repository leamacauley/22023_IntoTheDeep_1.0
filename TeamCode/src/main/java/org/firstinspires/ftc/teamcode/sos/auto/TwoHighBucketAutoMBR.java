package org.firstinspires.ftc.teamcode.sos.auto;




import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.sos.BaseRobot;

/**
 * Sample parking method using the Marist Base Robot Code
 * !!!NON-ROADRUNNER!!!
 */

@Autonomous(name="Two High Buckets", group="Auto")

public class TwoHighBucketAutoMBR extends LinearOpMode {
    // For Encoder Functions
    private double     COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private double     DRIVE_SPEED                   = 0.6;
    private double     TURN_SPEED                    = 0.5;

    /* Declare OpMode members. */
    BaseRobot robot   = new BaseRobot();
    private ElapsedTime period = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        waitForStart();

        robot.closeClaw();
        strafeInches(0.5,-23,300);
        pointTurnDegrees(0.5,120,300);

        robot.rotateArm(300,0.7);
        robot.waitForTick(700);
        robot.liftToPos(3100,0.8);

        robot.waitForTick(300);

        robot.rotateExtender(2400,1);

        driveStraightInches(0.3,10,300);
        robot.waitForTick(300);
        robot.openClaw();
        robot.waitForTick(1000);
        driveStraightInches(0.3,-13,300);
        robot.waitForTick(1000);

        robot.shoulder.setPosition(0.0);
        robot.rotateExtender(-100,0.9);
        robot.waitForTick(1000);
        robot.rotateArm(0,0.4);
        robot.liftToPos(0,0.8);
        robot.waitForTick(2000);

        pointTurnDegrees(0.6,254,300);
        driveStraightInches(0.6,3,300);
        robot.lowerIntake();
        robot.runIntake();
        robot.waitForTick(3000);
        driveStraightInches(0.4,-9,300);
        robot.stopIntake();
        robot.waitForTick(1000);

        robot.transfer();
        robot.waitForTick(1000);
        robot.runIntake();
        robot.waitForTick(1000);
        robot.stopIntake();

        driveStraightInches(0.5,4,300);
        robot.closeClaw();
        pointTurnDegrees(0.6,-253,300);

        driveStraightInches(0.5,2,300);

        robot.rotateArm(300,0.7);
        robot.waitForTick(700);
        robot.liftToPos(3100,0.8);

        robot.waitForTick(300);

        robot.rotateExtender(2400,1);

        driveStraightInches(0.3,9,300);
        robot.waitForTick(300);
        robot.openClaw();
        robot.waitForTick(1000);
        driveStraightInches(0.3,-6,300);
        robot.waitForTick(1000);

        robot.shoulder.setPosition(0.0);
        robot.rotateExtender(-100,0.9);
        robot.waitForTick(1000);
        robot.rotateArm(0,0.4);
        robot.liftToPos(0,0.8);
        robot.waitForTick(2000);



        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();


    }   // end runOpMode()


    public void strafeInches(double speed,
                             double inches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        inches = inches * -1 * (12.0/29);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > 0.9) {
            speed = 0.9; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = robot.rightRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

            //
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.leftRear.setTargetPosition(newLeftRearTarget);
            robot.rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftRear.setPower(Math.abs(speed));
            robot.rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftRear.isBusy() && robot.rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }


    }
    public void pointTurnDegrees(double speed,
                                 double deg,
                                 double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        deg = deg * -1;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
            newLeftRearTarget = robot.leftRear.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightRearTarget = robot.rightRear.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);

            //
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.leftRear.setTargetPosition(newLeftRearTarget);
            robot.rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftRear.setPower(Math.abs(speed));
            robot.rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftRear.isBusy() && robot.rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }
    public void driveStraightInches(double speed,
                                    double inches,
                                    double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        inches = inches * -1;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = robot.rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            //
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.leftRear.setTargetPosition(newLeftRearTarget);
            robot.rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftRear.setPower(Math.abs(speed));
            robot.rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftRear.isBusy() && robot.rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}   // en