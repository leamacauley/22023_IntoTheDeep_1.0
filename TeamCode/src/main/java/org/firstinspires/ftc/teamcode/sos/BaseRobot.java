package org.firstinspires.ftc.teamcode.sos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DigitalChannel;


    public class BaseRobot {
        /* Public Motors and Servos */
        public DcMotor leftFront = null;
        public DcMotor rightFront = null;
        public DcMotor leftRear = null;
        public DcMotor rightRear = null;

        public DcMotor rightLift = null;
        public DcMotor leftLift = null;

        public DcMotor arm = null;

        public Servo shoulder = null;

        public Servo pivot = null;

        public Servo intake = null;
        public Servo claw = null;

        public DcMotor extender = null;

        /* Public Sensors */

        public NormalizedColorSensor colorSensor = null;


        // For Encoder Functions
        private double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
        private final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        private double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        private double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        private double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;
        private double DRIVE_SPEED = 0.6;
        private double TURN_SPEED = 0.5;

        // Local OpMode members
        HardwareMap hwMap = null;
        private ElapsedTime period = new ElapsedTime();

        // Constructor - leave this blank for now
        public BaseRobot() {

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
            leftFront = hwMap.dcMotor.get("leftFront");
            rightFront = hwMap.dcMotor.get("rightFront");
            leftRear = hwMap.dcMotor.get("leftRear");
            rightRear = hwMap.dcMotor.get("rightRear");

            arm = hwMap.dcMotor.get("intakeArm");

            rightLift = hwMap.dcMotor.get("rightSlide");
            leftLift = hwMap.dcMotor.get("leftSlide");

            shoulder = hwMap.servo.get("shoulder");

            intake = hwMap.servo.get("intake");
            claw = hwMap.servo.get("claw");
            extender = hwMap.dcMotor.get("extender");

            pivot = hwMap.servo.get("pivot");

            colorSensor = hwMap.get(NormalizedColorSensor.class, "color");


            //colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");

            // motor directions
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);

            leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
            rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

            arm.setDirection(DcMotorSimple.Direction.FORWARD);


            // Set all motors to zero power
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            extender.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        /**
         * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
         * periodic tick.  This is used to compensate for varying processing times for each cycle.
         * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
         *
         * @param periodMs Length of wait cycle in mSec.
         **/
        public void waitForTick(long periodMs) {

            long remaining = periodMs - (long) period.milliseconds();

            // sleep for the remaining portion of the regular cycle period.
            if (remaining > 0) {
                try {
                    Thread.sleep(remaining);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

            // Reset the cycle clock for the next pass.
            period.reset();
        }

        /**
         * Lifts slider to position
         *
         * @param pos
         * @param speed
         */
        public void liftToPos(int pos, double speed) {
            rightLift.setTargetPosition(pos);
            leftLift.setTargetPosition(pos);
            if(pos < 0) {
                pos = 0;
            }
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(speed);
            leftLift.setPower(speed);
        }



        public void runIntake() {
            intake.setPosition(0.0);
        }

        public void openClaw() {
            claw.setPosition(0.2);
        }

        public void closeClaw() {
            claw.setPosition(0.0);
        }

        public void stopIntake() {
            intake.setPosition(0.5);
        }

        public void rotateArm(int pos, double speed) {
            arm.setTargetPosition(pos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(speed);
        }
         /* Determines if there's something blue, yellow, or red covering the sensor.
         * @return
         */

        public float getIntensity() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);
            return hsvValues[2];
        }



        // AUTOMATIONS
        public void lowerIntake() {
            rotateArm(820,0.4);
            shoulder.setPosition(0.45);
        }

        public void raiseIntake() {
            rotateArm(0,0.4);
            shoulder.setPosition(0.0);
        }

        /**
         * Set intake arm to vertical
         * Set intake pan to all the way back on the arm
         * Set arm to 0
         * Set sliders to 0
         */
        public void resetToZero() {
            leftLift.setTargetPosition(0);
            rightLift.setTargetPosition(0);
            //telemetry.addData("Say", "SlidePos: " + slidePos);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(0.8);
            rightLift.setPower(0.8);

            rotateArm(200,0.4);
            waitForTick(300);

            shoulder.setPosition(0.0);
            rotateExtender(-100,0.9);
            waitForTick(1000);
            rotateArm(0,0.4);
        }

        public void getSpecFromWall() {
            rotateArm(300,0.6);
            waitForTick(1300);
            rotateExtender(3000,1);
            waitForTick(300);
            //liftToPos(0,0.7);
        }

        public void rotateExtender(int pos, double speed) {
            extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extender.setTargetPosition(pos);
            extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extender.setPower(0.8);
        }


        public void transfer() {
            shoulder.setPosition(0.3);
            rotateArm(-250,0.4);
        }

        /**
         * lifts slider up
         * brings extender around
         */
        public void setScoringPos() {
            rotateArm(200,0.6);
            waitForTick(300);
            rotateExtender(2400,1);
        }
    }



