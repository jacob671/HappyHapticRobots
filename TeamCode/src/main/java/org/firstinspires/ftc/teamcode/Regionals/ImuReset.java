import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@TeleOp(name = "IMU Reset To Zero Example", group = "Sensor")
public class ImuReset extends LinearOpMode {
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware map to get the IMU sensor
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize the IMU with default parameters
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Run the OpMode
        while (opModeIsActive()) {
            // Set IMU to zero heading by initializing with adjusted parameters
            imu.resetYaw(); // Assuming resetYaw() method is available

            telemetry.addData("Status", "IMU Reset To Zero");
            telemetry.update();

            // Sleep for a short period to avoid overwhelming the IMU
            sleep(1000);
        }
    }
}
