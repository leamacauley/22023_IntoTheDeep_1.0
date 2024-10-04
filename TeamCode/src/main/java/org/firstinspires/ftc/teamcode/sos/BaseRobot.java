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
        public DcMotor leftFront   = null;
        public DcMotor rightFront  = null;
        public DcMotor leftRear    = null;
        public DcMotor rightRear   = null;

        /* Public Sensors */
        public DigitalChannel touch = null;
        public NormalizedColorSensor colorSensor = null;


        // For Encoder Functions
        private double     COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
        private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        private double     WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
        private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
        private double     DRIVE_SPEED                   = 0.6;
        private double     TURN_SPEED                    = 0.5;

        // Local OpMode members
        HardwareMap hwMap = null;
        private ElapsedTime period = new ElapsedTime();

        // Constructor - leave this blank for now
        public BaseRobot () {

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
            leftFront   = hwMap.dcMotor.get("leftfront");
            rightFront  = hwMap.dcMotor.get("rightfront");
            leftRear     = hwMap.dcMotor.get("leftrear");
            rightRear    = hwMap.dcMotor.get("rightrear");

            colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");

            // motor directions
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /**
         *
         * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
         * periodic tick.  This is used to compensate for varying processing times for each cycle.
         * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
         *
         * @param periodMs  Length of wait cycle in mSec.
         **/
        public void waitForTick(long periodMs) {

            long  remaining = periodMs - (long)period.milliseconds();

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
         * Uses color sensor to determine if there's a sample in the intake
         */
        public boolean sensorBlocked() {
            float hue = this.getHue();
            telemetry.addData ("Average hue", hue);
            telemetry.update();

            return true;
        }
        public float [] getColorValues() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float [] output = new float [3];
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

