package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Harderware {
    /* Public OpMode members. */
    DcMotor leftFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor liftMotor = null;
    DcMotor holdMotor = null;
    DcMotor catapultMotor = null;
    Servo basketServo = null;
    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device
/*
    DcMotor shootingMotor = null;
    DcMotor sweeperMotor = null;

    Servo buttonServo = null;
*/

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Harderware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("leftFrontMotor");   // C1M1
        leftBackMotor = hwMap.dcMotor.get("leftBackMotor");     // C1M2
        rightFrontMotor = hwMap.dcMotor.get("rightFrontMotor"); // C2M1
        rightBackMotor = hwMap.dcMotor.get("rightBackMotor");   // C2M2
        liftMotor = hwMap.dcMotor.get("liftMotor");
        holdMotor = hwMap.dcMotor.get("holdMotor");
        catapultMotor = hwMap.dcMotor.get("catapultMotor");
        basketServo = hwMap.servo.get("basketServo"); //servo
        /*
        shootingMotor = hwMap.dcMotor.get("shootingMotor");
        sweeperMotor = hwMap.dcMotor.get("sweeperMotor");

        buttonServo = hwMap.servo.get("buttonServo");   //servo
        */
        // 1. Set all motors to run with encoders
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 2. Set all DC motors to proper rotation direction
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        holdMotor.setDirection(DcMotor.Direction.FORWARD);
        catapultMotor.setDirection(DcMotor.Direction.FORWARD);

        /*
        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);
        */

        catapultMotor.setMaxSpeed(375);

        // 3. Set all motors to ZERO power
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        liftMotor.setPower(0);
        holdMotor.setPower(0);
        catapultMotor.setPower(0);

        /*
        holdM
        shootingMotor.setPower(0);
        sweeperMotor.setPower(0);
        */

    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
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
}
