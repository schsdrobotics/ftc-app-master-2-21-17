package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Holodrive", group="2017")
public class holoDrive extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    Harderware robot = new Harderware();    // gets hardware property configuration
                                            // from hardware class


    static final double VERTICAL_COEFF = 1.0;   // assign constants for altering values for
    static final double HORIZONTAL_COEFF = 1.0; // degrees of motion
    static  final double TURN_COEFF = 1.0;

    double leftFrontPower;
    double leftBackPower;          //declare power value variables outside of main loop
    double rightFrontPower;
    double rightBackPower;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);        //get hardware configuration from phones

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        int controlToggle = -1;    // either 1 or -1, used to reverse "front" orientation
                                    // of robot




        // loop until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            float j1x = gamepad1.left_stick_x;
            float j1y = -gamepad1.left_stick_y;     //get values from joysticks
            float j2x = gamepad1.right_stick_x;


            //multiplies each joystick value by corresponding matrix value for each degree of motion
            //
            //
            //   horizontal:              |     vertical:                     |     rotation:
            //      -1/√2  ,    -1/√2     |          -1/√2     ,     1/√2     |         1   ,    1
            //                            |                                   |
            //      1/√2   ,     1/√2     |          -1/√2      ,    1/√2     |         1   ,    1
            //                            |
            //  uses rounded value of 1/√2 as calculating it takes too much processing power
            leftFrontPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
            rightFrontPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
            leftBackPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
            rightBackPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);

            Range.clip(leftFrontPower, -1, 1);
            Range.clip(rightFrontPower, -1, 1);  // clips values as motor power cannot
            Range.clip(leftBackPower, -1, 1);    // go above 1 or below -1
            Range.clip(rightBackPower, -1, 1);

            robot.leftFrontMotor.setPower(leftFrontPower);
            robot.rightFrontMotor.setPower(rightFrontPower);  //assigns finalized values to motors
            robot.leftBackMotor.setPower(leftBackPower);
            robot.rightBackMotor.setPower(rightBackPower);




            telemetry.addData("Control Direction", controlToggle);  //writes value of controlToggle
            idle();                                                 // to telemetry



        }

    }

}
