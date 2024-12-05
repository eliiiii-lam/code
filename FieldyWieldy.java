package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp

public class FieldyWieldy extends LinearOpMode {


    boolean slowMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //testinggggg
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bR");


        CRServo spinnyL = hardwareMap.crservo.get("spinnyL");
        CRServo spinnyR = hardwareMap.crservo.get("spinnyR");


        DcMotor uppiesL = hardwareMap.dcMotor.get("uppiesL");
        DcMotor uppiesR = hardwareMap.dcMotor.get("uppiesR");


        Servo inL = hardwareMap.servo.get("inL");
        Servo inR = hardwareMap.servo.get("inR");


        Servo linkL = hardwareMap.servo.get("linkL");
        Servo linkR = hardwareMap.servo.get("linkR");

        Servo clawL = hardwareMap.servo.get("clawL");
        Servo clawR = hardwareMap.servo.get("clawR");

        Servo uppiesJointL = hardwareMap.servo.get("uppiesJointL");
        Servo uppiesJointR = hardwareMap.servo.get("uppiesJointR");








        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);




        inL.setDirection(Servo.Direction.REVERSE);
        linkL.setDirection(Servo.Direction.REVERSE);


        spinnyR.setDirection(CRServo.Direction.REVERSE);

        uppiesJointR.setDirection(Servo.Direction.REVERSE);

        clawR.setDirection(Servo.Direction.REVERSE);

        spinnyL.setPower(0);
        spinnyR.setPower(0);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {






            double y = -gamepad1.left_stick_y *  0.6; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x  * 0.6;
            double rx = -gamepad1.right_stick_x * 0.6;




            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            if (Math.abs(gamepad2.left_stick_y) > 0.1){
                uppiesL.setPower(gamepad2.left_stick_y * 1);
                uppiesR.setPower(gamepad2.left_stick_y * 1);
            } else {
                uppiesL.setPower(0);
                uppiesR.setPower(0);
            }


            if (gamepad2.b){
                uppiesJointL.setPosition(0.07);
                uppiesJointR.setPosition(0.07);
            } else if (gamepad2.y){
               uppiesJointL.setPosition(0.3);
               uppiesJointR.setPosition(0.3);
            } else {
                uppiesJointL.setPosition(0.7);
                uppiesJointR.setPosition(0.7);
            }

            if (gamepad2.left_bumper){
                clawL.setPosition(0.5);
                clawR.setPosition(0.5);
                spinnyL.setPower(-1);
                spinnyR.setPower(-1);
            } else {
                clawL.setPosition(0.4);
                clawR.setPosition(0.4);
            }

            if (gamepad2.a){
                linkL.setPosition(0.5);
                linkR.setPosition(0.5);
            } else {
                linkL.setPosition(0.7);
                linkR.setPosition(0.7);
            }




            if (gamepad2.x){
                inL.setPosition(0.2);
                inR.setPosition(0.2);

            } else {
                inL.setPosition(0.85);
                inR.setPosition(0.85);
            }





            if (gamepad2.dpad_down){
                spinnyL.setPower(1);
                spinnyR.setPower(1);
            }

            if (gamepad2.dpad_up){
                spinnyL.setPower(-1);
                spinnyR.setPower(-1);
            }





            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}