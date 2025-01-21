package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem {

    // Elevator motor
    private DcMotor upLeftMotor;

    // Claw servos
    private Servo spinServo;
    private Servo turnServo;
    private Servo extendServo;
    private Servo bucketServo;

    // Constants for elevator power
    private static final double ELEVATOR_POWER = 1.0;
    private static final int TICKS_PER_INCH = 100; // Adjust based on your motor and gearing
    private static final int FULL_LIFT_TICKS = 4300; // Total ticks for full lift height
    private static final int SMALL_LIFT_TICKS = 2900; // Ticks for small lift height
    private static final int BIT_LIFT_TICKS = 800; // Ticks for small adjustments
    private static final int SPICIMENT_RELEASE_TICKS = 1900; // Ticks for small adjustments
    private static final int SPICIMENT_AFTER_RELEASE_TICKS = 2300; // Ticks for small adjustments

    private static final int STARTING_POSITION_TICKS = 0; // Define the starting position

    // Servo positions
    private static final double TURN_GRAB_POSITION = -;
    private static final double TURN_RELEASE_POSITION = 0.5;
    private static final double SPIN_GRAB_POSITION = 0.63;
    private static final double SPIN_RELEASE_POSITION = 1.2;

    private static final double BUCKET_DUMP_POSITION = 0.1;
    private static final double BUCKET_NEUTRAL_POSITION = 0.5;
    private static final double EXTEND_ORIGINAL_POSITION = 0.2;

    // Current state tracking
    private boolean liftControlActive = false;
    public int height = 0; // 0 = down, 1 = specimen height, 2 = full elevator height

    public ClawSubsystem(HardwareMap hardwareMap) {
        // Initialize elevator motor
        upLeftMotor = hardwareMap.get(DcMotor.class, "UpLeftMotor");
        upLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize claw servos
        spinServo = hardwareMap.get(Servo.class, "SpinServo");
        turnServo = hardwareMap.get(Servo.class, "TurnServo");
        extendServo = hardwareMap.get(Servo.class, "ExtendServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
    }

    // Elevator Controls
    public void moveUp() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevator(-FULL_LIFT_TICKS, 2); // Set height to 2 when fully up
        }
    }

    public void moveDown() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(STARTING_POSITION_TICKS, 0); // Set height to 0 when fully down
        }
    }

    public void moveUpSp() {
        if (!liftControlActive) {
            liftControlActive = true;
            //moveElevator(-SMALL_LIFT_TICKS, 1); // Set height to 1 for small lift
            moveElevatorToPosition(-SMALL_LIFT_TICKS, 1);
        }
    }

    public void moveUpSp_withMoveElevator() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevator(-SMALL_LIFT_TICKS, 1); // Set height to 1 for small lift
            //moveElevatorToPosition(-SMALL_LIFT_TICKS, 1);
        }
    }

    public void moveUpSpDownBit() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(-SPICIMENT_RELEASE_TICKS, 9); // Set height to 1 for small lift

            // moveElevator(500, 9); // Set height to 1 for small lift
            //moveElevatorToPosition(-SMALL_LIFT_TICKS + BIT_LIFT_TICKS, 9);

        }
    }


    public void moveUpSpDownBitThenUp() {
        if (!liftControlActive) {
            liftControlActive = true;
            //moveElevator(-SMALL_LIFT_TICKS, 11); // Set height to 1 for small lift
            // moveElevator(-100, 11); // Set height to 1 for small lift
            //moveElevatorToPosition(-SMALL_LIFT_TICKS, 11);
            moveElevatorToPosition(-SPICIMENT_AFTER_RELEASE_TICKS, 11);

        }
    }


    public void moveDownSp() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(STARTING_POSITION_TICKS, 0); // Set height to 0 when back to start
        }
    }
    public void extend() {
        extendServo.setPosition(0.8);

    }
    public void retract() {
        extendServo.setPosition(0.5);

    }






    public void moveBit() {
        int currentPosition = upLeftMotor.getCurrentPosition();
        int targetDownPosition = currentPosition + BIT_LIFT_TICKS;

        moveElevatorToPosition(targetDownPosition, height);

        // After reaching the lowered position, return to the original position
        new Thread(() -> {
            while (upLeftMotor.isBusy()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            moveElevatorToPosition(currentPosition, height);
        }).start();
    }
    public int getElevatorPosition(){
        return(upLeftMotor.getCurrentPosition());
    }

    private void moveElevator(int ticks, int targetHeight) {
        int targetPosition = upLeftMotor.getCurrentPosition() + ticks;
        upLeftMotor.setTargetPosition(targetPosition);
        upLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upLeftMotor.setPower(ELEVATOR_POWER);

        new Thread(() -> {
            while (upLeftMotor.isBusy()) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            // Check if the current position is close enough to the target position
            if (Math.abs(upLeftMotor.getCurrentPosition() - targetPosition) <= 20) {
                height = targetHeight; // Update height only if the target position is reached
            } else {
                // Handle or log failure to reach target position
                System.out.println("Failed to reach the target position.");
            }

            stopElevator();
        }).start();
    }



    private void moveElevatorToPosition(int targetPosition, int targetHeight) {
        upLeftMotor.setTargetPosition(targetPosition);
        upLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upLeftMotor.setPower(ELEVATOR_POWER);

        new Thread(() -> {
            while (upLeftMotor.isBusy()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

            // Check if the current position matches the target position
            if (Math.abs(upLeftMotor.getCurrentPosition() - targetPosition) <= 20) {
                height = targetHeight; // Update height only if the target position is reached
            } else {
                // Log or handle a failure to reach the target position, if needed
                System.out.println("Target position not reached!");
            }

            stopElevator();
        }).start();
    }


    public void stopElevator() {
        upLeftMotor.setPower(0);
        upLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftControlActive = false;
    }

    // Claw and bucket controls
    public void bucket() {
        bucketServo.setPosition(BUCKET_DUMP_POSITION);
        delayAction(() -> bucketServo.setPosition(BUCKET_NEUTRAL_POSITION), 1000);
    }

    public void grab() {
        turnServo.setPosition(TURN_GRAB_POSITION);
        delayAction(() -> spinServo.setPosition(SPIN_GRAB_POSITION), 100);
    }

    public void grabReady() {
        spinServo.setPosition(SPIN_RELEASE_POSITION);
        turnServo.setPosition(1);
    }

    public void grabVertical() {
        extendServo.setPosition(EXTEND_ORIGINAL_POSITION);
        turnServo.setPosition(0.5);


    }

    public void reset() {
        turnServo.setPosition(0.9);

        spinServo.setPosition(SPIN_RELEASE_POSITION);

    }

    public void release() {
        turnServo.setPosition(TURN_RELEASE_POSITION);
        delayAction(() -> spinServo.setPosition(SPIN_RELEASE_POSITION), 700);
    }

    public void extendOriginal() {
        extendServo.setPosition(EXTEND_ORIGINAL_POSITION);
        turnServo.setPosition(0.5);
    }

    // Utility to delay actions
    private void delayAction(Runnable action, long delay) {
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                action.run();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }
}
