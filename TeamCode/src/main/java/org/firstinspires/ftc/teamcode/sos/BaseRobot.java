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

        public Servo wrist = null;

        public Servo intake = null;
        public Servo claw = null;

        public Servo hinge = null;

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
            leftFront = hwMap.dcMotor.get("leftfront");
            rightFront = hwMap.dcMotor.get("rightfront");
            leftRear = hwMap.dcMotor.get("leftrear");
            rightRear = hwMap.dcMotor.get("rightrear");

            arm = hwMap.dcMotor.get("arm");

            rightLift = hwMap.dcMotor.get("rightlift");
            leftLift = hwMap.dcMotor.get("leftlift");

            shoulder = hwMap.servo.get("shoulder");
            wrist = hwMap.servo.get("wrist");

            intake = hwMap.servo.get("intake");
            claw = hwMap.servo.get("claw");
            hinge = hwMap.servo.get("hinge");

            colorSensor = (NormalizedColorSensor) hwMap.colorSensor.get("color");


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

            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            rightLift.setPower(speed);
            leftLift.setPower(speed);
            rightLift.setTargetPosition(pos);
            leftLift.setTargetPosition(pos);
            if(pos < 0) {
                pos = 0;
            }
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void runIntake() {
            intake.setPosition(0.0);
        }
        public void automateTransfer() {

        }

        public void openClaw() {
            claw.setPosition(0.1);
        }

        public void stopIntake() {
            intake.setPosition(0.5);
        }

        public void rotateArm(int pos, double speed) {
            arm.setPower(speed);
            if(pos < 0) {
                pos = 0;
            }
            arm.setTargetPosition(pos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void rotateShoulder(double pos) {
            shoulder.setPosition(pos);
        }

        public void rotateWrist(double pos) {
            wrist.setPosition(pos);
        }



        public void grabFromCenterWithSensor() {
            if (sensorBlocked()) {  // make this simultaneous
                stopIntake();
                // add
            }
            else {
                runIntake();
                // add
            }
        }


         /* Determines if there's something blue, yellow, or red covering the sensor.
         * @return
         */
        public boolean sensorBlocked() {
            float[] colorValues = getColorValues();
            double red = colorValues[0];
            double blue = colorValues[1];
            double green = colorValues[2];

            // Define thresholds to detect colors
            double redThreshold = 0.3;  // Example threshold for red
            double blueThreshold = 0.3; // Example threshold for blue
            double greenThreshold = 0.3; // Example threshold for green

            // You may need to adjust these thresholds based on your sensor and conditions

            // Check if any color value is above the threshold
            if (red > redThreshold && red > blue && red > green) {
                return true;  // Red color detected
            } else if (blue > blueThreshold && blue > red && blue > green) {
                return true;  // Blue color detected
            } else if (green > greenThreshold && green > red && green > blue) {
                return true;  // Yellow color detected (because yellow is a mix of red and green)
            }

            return false;  // No color detected or not in the threshold range
        }


        public float[] getColorValues() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] output = new float[3];
            output[0] = colors.red;
            output[1] = colors.blue;
            output[2] = colors.green;
            return output;
        }

        public double getRed() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            return colors.red;
        }

        public double getBlue() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            return colors.blue;
        }

        public float getIntensity() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);
            return hsvValues[1];
        }

        public float getHue() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);
            return hsvValues[0];
        }


    }



