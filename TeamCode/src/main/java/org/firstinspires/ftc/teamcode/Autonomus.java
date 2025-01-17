package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Autonomous(name = "Autonomous Mode Mecanum", group = "Linear Opmode")
public class Autonomus extends LinearOpMode {
    private int RinitialPosition;
    private int LinitialPosition;
    public void init(HardwareMap hardwareMap) {


        UpLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        UpLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        UpRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        UpRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);// Set the initial position
        LinitialPosition = UpLeftMotor.getCurrentPosition();
        RinitialPosition = UpRightMotor.getCurrentPosition();
    }
    private Servo bucketServo;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor UpLeftMotor;
    private DcMotor UpRightMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo ExtendServo;
    private Servo TurnServo;
    private Servo SpinServo;

    private final double MAX_POSITION = 1.0;
    private final double MIN_POSITION = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware variables
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        UpRightMotor = hardwareMap.get(DcMotor.class, "UpRightMotor");
        UpLeftMotor = hardwareMap.get(DcMotor.class, "UpLeftMotor");
        SpinServo = hardwareMap.get(Servo.class, "SpinServo");
        ExtendServo = hardwareMap.get(Servo.class, "ExtendServo");
        TurnServo = hardwareMap.get(Servo.class, "TurnServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        // Reverse the right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        // 90 degrees is = turn(.8, 378);
        // Autonomous actions

        UpLeftMotor.setPower(-1);
        UpRightMotor.setPower(1);
        sleep(2000);
        UpLeftMotor.setPower(0);
        UpRightMotor.setPower(0);

        moveForward(.8, 310);  // Move forward for 2 second
        sleep(500);
        strafeLeft(0.4, 602);
        sleep(500);
        moveForward(0.4, 260);  // Move forward for 2 second
        sleep(560);
        TurnServo.setPosition(.55);

        UpLeftMotor.setPower(1);
        UpRightMotor.setPower(-1);
        sleep(1900);

        UpLeftMotor.setPower(0);
        UpRightMotor.setPower(0);

        sleep(500);
        moveBackward(0.4, 1403);
        sleep(200);
        moveForward(0.8, 300);

        turn(0.8, 540);
        moveForward(0.4, 1060);
        sleep(500);
        moveBackward(0.4, 500);
        sleep(1000);
        moveForward(0.4, 650);
        sleep(500);
        UpLeftMotor.setPower(-1);
        UpRightMotor.setPower(1);
        sleep(900);
        UpLeftMotor.setPower(0);
        UpRightMotor.setPower(0);
        moveBackward(0.8, 350);
        turn(-0.8, 385);
        moveBackward(.8, 900);
        turn(-.8, 443);
        UpLeftMotor.setPower(-1);
        UpRightMotor.setPower(1);
        sleep(1000);
        UpLeftMotor.setPower(0);
        UpRightMotor.setPower(0);
        moveBackward(0.4, 1353);
        moveForward(0.4, 1140);
        sleep(1900);

        UpLeftMotor.setPower(.8);
        UpRightMotor.setPower(-.8);
        sleep(2350);

        UpLeftMotor.setPower(0);
        UpRightMotor.setPower(0);



        moveBackward(0.8, 500);
        sleep(430);
        moveForward(0.8, 60);
        turn(-.8, 398);
        moveBackward(0.8, 720);
        ExtendServo.setPosition(.7);

        TurnServo.setPosition(.8);
        sleep(200);









        //finished first specimen

        /*
        sleep(200);

        moveForward(1, 200);
        sleep(200);
        frontLeftMotor.setPower(-0.5*0.5);
        backRightMotor.setPower(-0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5*0.5);
        turn(0.5,1000);
        sleep(3000);
        stopMotors();
        turn(0.4, 1);
        sleep(200);
        TurnServo.setPosition(0.9);
        sleep(500);
        SpinServo.setPosition(0.59);
        sleep(1000);
        TurnServo.setPosition(0.3);
        sleep(1000);
        SpinServo.setPosition(0);
        UpLeftMotor.setPower(-1);
        UpRightMotor.setPower(1);
        sleep(3000);
        UpLeftMotor.setPower(0);
        UpRightMotor.setPower(0);
        bucketServo.setPosition(0);
            /*
        sleep(1000);
        bucketServo.setPosition(0.5);
        sleep(1000);
        resetLift();
        */
    }



    public void resetLift() {
        UpRightMotor.setTargetPosition(RinitialPosition);
        UpRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        UpLeftMotor.setTargetPosition(LinitialPosition);
        UpLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        UpLeftMotor.setPower(1);
        UpRightMotor.setPower(-1);
        while (UpRightMotor.isBusy() && UpLeftMotor.isBusy()) { // Wait until the motor reaches the initial position
        }
        UpLeftMotor.setPower(0);
        UpRightMotor.setPower(0);
        UpLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        UpRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to strafe right
    private void strafeRight(double power, long duration) {
        frontLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        sleep(duration);
        stopMotors();
    }

    private void turn(double power, long duration) {
        frontLeftMotor.setPower(power);
        backRightMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(power);
        sleep(duration);
        stopMotors();
    }
    private void moveForward(double power, long duration) {
        setMotorPowers(power, power, power, power);
        sleep(duration);
        stopMotors(); }
    private void moveBackward(double power, long duration) {
        setMotorPowers(-power, -power, -power, -power);
        sleep(duration);
        stopMotors(); }

    // Method to strafe left
    private void strafeLeft(double power, long duration) {
        frontLeftMotor.setPower(-power);
        backRightMotor.setPower(-0.5*power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(0.5*power);
        sleep(duration);
        stopMotors();
    }

    // Method to spin the servos
    private void spinServos(double position, long duration) {
        SpinServo.setPosition(position);
        sleep(duration);
        SpinServo.setPosition(0.0); // Reset position after spinning
    }

    // Method to move lift
    private void moveLift(double power, long duration) {
        UpLeftMotor.setPower(power);
        UpRightMotor.setPower(-power);
        sleep(duration);
        UpLeftMotor.setPower(0);
        UpRightMotor.setPower(0);
    }

    // Set power to all motors
    private void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        backLeftMotor.setPower(backLeft);
        frontRightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
    }

    // Stop all motors
    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    // Method to set arm servo position






}