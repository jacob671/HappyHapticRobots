import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;


import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name = "TeleOp", group = "Linear Opmode")
public class Tesy extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor UpLeftMotor;
    private DcMotor UpRightMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo ExtendServo;
    private Servo TurnServo;

    private ColorSensor colorSensor;
    private Servo SpinServo;
    private Servo bucketServo;
    private Servo Speciman;
    private double leftArmPosition = 0.5;
    private double Height = 1;
    private double HeightSpec = 1;
    private double HeightSpec1 = 0;
    private double rightArmPosition = 1;
    private double divisor = 1;

    private double currentPosition = 0.2;
    private Timer timer = new Timer();
    private boolean liftControlActive = false;
    private boolean bucketControlActive = false;


    public void runOpMode() {
        // Initialize hardware variables
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        UpRightMotor = hardwareMap.get(DcMotor.class, "UpRightMotor");
        UpLeftMotor = hardwareMap.get(DcMotor.class, "UpLeftMotor");
        SpinServo = hardwareMap.get(Servo.class, "SpinServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        ExtendServo = hardwareMap.get(Servo.class, "ExtendServo");
        TurnServo = hardwareMap.get(Servo.class, "TurnServo");


        // Reverse the right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Get gamepad values
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double y2 = gamepad2.left_stick_y;
            double denominator = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rx))*divisor, 1*divisor);





            // Calculate motor powers


            double frontLeftPower = (y + x + rx) / denominator;
            double ExtendAmount = (y2)/ denominator/10;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Read color sensor values


            // Set motor powers


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Control the lift motors
            if (gamepad2.left_trigger > 0.4 && !liftControlActive) {
                liftControlActive = true;
                if (Height == 0) {
                    divisor=1;
                    rightArmPosition = .4;
                    TurnServo.setPosition(rightArmPosition);
                    UpLeftMotor.setPower(1);
                    UpRightMotor.setPower(-1);
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            UpLeftMotor.setPower(0);
                            UpRightMotor.setPower(0);
                            Height = 1;


                            liftControlActive = false;
                        }
                    }, 3100);
                }
            } else if (gamepad2.right_trigger > 0.4 && !liftControlActive) {
                liftControlActive = true;
                if (Height == 1) {

                    UpLeftMotor.setPower(-1);
                    UpRightMotor.setPower(1);
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            UpLeftMotor.setPower(0);
                            UpRightMotor.setPower(0);
                            Height = 0;
                            liftControlActive = false;
                            divisor=2;
                        }
                    }, 3250);

                }
            }

            if (gamepad2.x && !bucketControlActive) {
                bucketControlActive = true;
                bucketServo.setPosition(0.14);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        bucketServo.setPosition(0.5);
                        bucketControlActive = false;
                    }
                }, 1000);
            }


            // Control the servos with triggers
            if (gamepad1.right_trigger > 0.51) {
                divisor=4;

                leftArmPosition = 0.8;
                rightArmPosition = .3;
                currentPosition = 0.7;
                ExtendServo.setPosition(currentPosition);
                sleep(200);
                TurnServo.setPosition(.82);
                SpinServo.setPosition(0);

            } else if (gamepad1.left_trigger > 0.51) {
                divisor=1;
                currentPosition = 0.2;

                ExtendServo.setPosition(currentPosition);
                TurnServo.setPosition(0.35);
                sleep(650);

                SpinServo.setPosition(0);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        SpinServo.setPosition(0);
                    }
                }, 1000);

            }

            if (gamepad1.y) {
                divisor=1;
                TurnServo.setPosition(0.9);
                SpinServo.setPosition(0.7);
                // Log the values to telemetry

            }

            if (gamepad1.b) {
                divisor=4;
                SpinServo.setPosition(0);
                TurnServo.setPosition(0.75);

            }

            if (gamepad2.y) {
                Speciman.setPosition(0.3);


            }
            if (gamepad2.b) {
                Speciman.setPosition(0);

            }
            if (gamepad2.right_bumper && !liftControlActive) {
                liftControlActive = true;
                if (HeightSpec == 1) {
                    divisor=2;
                    UpLeftMotor.setPower(-1);
                    UpRightMotor.setPower(1);
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            UpLeftMotor.setPower(0);
                            UpRightMotor.setPower(0);
                            HeightSpec = 0;
                            liftControlActive = false;
                        }
                    }, 1900);
                }
            }

            if (gamepad2.left_bumper && !liftControlActive) {
                liftControlActive = true;
                if (HeightSpec == 0) {
                    divisor=1;
                    UpLeftMotor.setPower(1);
                    UpRightMotor.setPower(-1);
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            UpLeftMotor.setPower(0);
                            UpRightMotor.setPower(0);
                            HeightSpec = 1;
                            liftControlActive = false;
                        }
                    }, 1840);
                }
            }
            if (gamepad2.a) {
                liftControlActive = true;
                if (HeightSpec1 == 0) {
                    divisor=2;
                    UpLeftMotor.setPower(-1);
                    UpRightMotor.setPower(1);
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            UpLeftMotor.setPower(0);
                            UpRightMotor.setPower(0);
                            HeightSpec1 = 1;
                            liftControlActive = false;
                        }
                    }, 100);
                }
                else if (HeightSpec1 == 1) {
                    divisor = 1;
                    TurnServo.setPosition(0.2);
                    UpLeftMotor.setPower(1);
                    UpRightMotor.setPower(-1);
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            UpLeftMotor.setPower(0);
                            UpRightMotor.setPower(0);
                            HeightSpec1 = 0;
                            TurnServo.setPosition(0);
                            liftControlActive = false;
                        }
                    }, 90);
                }
            }





            // Add telemetry data
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);

            telemetry.update();
        }
    }
}
