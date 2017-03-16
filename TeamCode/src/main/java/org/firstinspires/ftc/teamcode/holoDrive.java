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

        robot.leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);     //set all to forward to
        robot.rightBackMotor.setDirection(DcMotor.Direction.FORWARD);      //allow for correct power
        robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);           //assignments
        robot.holdMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.catapultMotor.setDirection(DcMotor.Direction.FORWARD);

        robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

            if (controlToggle == -1){
                leftFrontPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
                rightFrontPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
                leftBackPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
                rightBackPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
            } else if (controlToggle == 1) {
                leftFrontPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
                rightFrontPower = (HORIZONTAL_COEFF * j1x * -0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
                leftBackPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * 0.7071) + (TURN_COEFF * j2x * 1);
                rightBackPower = (HORIZONTAL_COEFF * j1x * 0.7071) + (VERTICAL_COEFF * j1y * -0.7071) + (TURN_COEFF * j2x * 1);
            }




            Range.clip(leftFrontPower, -1, 1);
            Range.clip(rightFrontPower, -1, 1);  // clips values as motor power cannot
            Range.clip(leftBackPower, -1, 1);    // go above 1 or below -1c v
            Range.clip(rightBackPower, -1, 1);

            robot.leftFrontMotor.setPower(leftFrontPower);
            robot.rightFrontMotor.setPower(rightFrontPower);  //assigns finalized values to motors
            robot.leftBackMotor.setPower(leftBackPower);
            robot.rightBackMotor.setPower(rightBackPower);


            /////////////Single Button Controls/////////////

            if (gamepad2.right_bumper){
                robot.catapultMotor.setPower(1);
            } else {
                robot.catapultMotor.setPower(0);
            }

            if (gamepad1.x) {
                robot.sweeperServo.setPower(-1);

            } else if (gamepad1.y) {
                robot.sweeperServo.setPower(1);
            } else robot.sweeperServo.setPower(0);


            if (gamepad2.dpad_up){
                robot.liftMotor.setPower(1);
            } else if (gamepad2.dpad_down){
                robot.liftMotor.setPower(-1);
            } else {
                robot.liftMotor.setPower(0);
            }

            if (gamepad2.a){
                robot.holdMotor.setPower(1.0);
            } else if (gamepad2.b) {
                robot.holdMotor.setPower(-1.0);
            } else {
                robot.holdMotor.setPower(0);
            }


            if (gamepad2.y){
                robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.elevatorMotor.setPower(-1);
                while (!robot.topLimit.isPressed() && robot.elevatorMotor.getCurrentPosition() >= -2550);
                robot.elevatorMotor.setPower(0);
                sleep(1000);
                robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.elevatorMotor.setPower(1);
                while (!robot.bottomLimit.isPressed() && robot.elevatorMotor.getCurrentPosition() <= 2550);
                robot.elevatorMotor.setPower(0);


//            } else if (gamepad2.x) {
//                robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.elevatorMotor.setPower(0.5);
//                while (!robot.bottomLimit.isPressed() && robot.elevatorMotor.getCurrentPosition() <= 2450);
//                robot.elevatorMotor.setPower(0);
            }

            if(gamepad1.start){
                controlToggle = -1*controlToggle;
                sleep(250);
            }



            telemetry.addData("Control Direction", controlToggle);  //writes value of controlToggle
            idle();                                                 // to telemetry

//

        }

    }

}
