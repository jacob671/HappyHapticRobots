package org.firstinspires.ftc.teamcode.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;

@Autonomous(name = "Owen1", group = "Competition")
public class AutoOwen extends OpMode {
    private int x = 0;
    private int pathState = 0;
    private int highChamberCount = 0;
    private boolean elevatorUp = false;

    private Follower follower;
    private Timer opmodeTimer;
    private ClawSubsystem clawSubsystem;

    // Starting and target positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Pose placeSpecimenPose = new Pose(30, 12, 0);
    private Pose placeSpecimenPose1 = new Pose(35, 15, 0);
    private final Pose afterSpecimenPose = new Pose(12, -17.4, 0);
    private final Pose specimen1Pose = new Pose(44, -26.4, 0);
    private final Pose specimen1StrafePose = new Pose(52, -30, 0);
    private final Pose specimen1PushPose = new Pose(14, -30, 0);
    private final Pose specimen2Pose = new Pose(25, -20, 0);
    private final Pose specimen2StrafePose = new Pose(58, -35, 0);
    private final Pose specimen2PushPose = new Pose(24, -35, 0);
    private final Pose readyForHuman = new Pose(10, -19.8, Math.toRadians(180));
    private final Pose humanPlayerZonePose = new Pose(-6, -19.8, Math.toRadians(180));
    private final Pose humanPlayerZonebackPose = new Pose(3, -19.8, Math.toRadians(180));
    private final Pose highChamberPose = new Pose(28, 14, 0);
    private final Pose returnHomePose = new Pose(0, 0, 0);

    // Paths
    private Path placeSpecimenPath, pushReadyPath, specimen1Path, specimen1StrafePath,
            specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath,
            readyForHumanPath1, getSpeciPath, getSpeciBackPath, specimenToHighChamberPath, returnHomePath;
    private PathChain pushPath;

    @Override
    public void init() {
        clawSubsystem = new ClawSubsystem(hardwareMap);
        follower = new Follower(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower.setStartingPose(startPose);
        //clawSubsystem.height = 0;
        buildPaths();
        clawSubsystem.moveDownSp();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        if(clawSubsystem.height != 0){
            clawSubsystem.moveDownSp();
        }

    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        Telemetry.Item elapsedTime = telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());

        telemetry.addData("elevator Height", clawSubsystem.height);
        telemetry.update();
    }

    private void buildPaths() {
        placeSpecimenPath = new Path(new BezierLine(new Point(startPose), new Point(placeSpecimenPose)));
        placeSpecimenPath.setLinearHeadingInterpolation(startPose.getHeading(), placeSpecimenPose.getHeading());

        pushReadyPath = new Path(new BezierLine(new Point(placeSpecimenPose), new Point(afterSpecimenPose)));
        pushReadyPath.setLinearHeadingInterpolation(placeSpecimenPose.getHeading(), afterSpecimenPose.getHeading());

        specimen1Path = new Path(new BezierLine(new Point(afterSpecimenPose), new Point(specimen1Pose)));
        specimen1Path.setLinearHeadingInterpolation(afterSpecimenPose.getHeading(), specimen1Pose.getHeading());

        specimen1StrafePath = new Path(new BezierLine(new Point(specimen1Pose), new Point(specimen1StrafePose)));
        specimen1StrafePath.setLinearHeadingInterpolation(specimen1Pose.getHeading(), specimen1StrafePose.getHeading());

        specimen1PushPath = new Path(new BezierLine(new Point(specimen1StrafePose), new Point(specimen1PushPose)));
        specimen1PushPath.setLinearHeadingInterpolation(specimen1StrafePose.getHeading(), specimen1PushPose.getHeading());

        specimen2Path = new Path(new BezierLine(new Point(specimen1PushPose), new Point(specimen2Pose)));
        specimen2Path.setLinearHeadingInterpolation(specimen1PushPose.getHeading(), specimen2Pose.getHeading());

        specimen2StrafePath = new Path(new BezierLine(new Point(specimen2Pose), new Point(specimen2StrafePose)));
        specimen2StrafePath.setLinearHeadingInterpolation(specimen2Pose.getHeading(), specimen2StrafePose.getHeading());

        specimen2PushPath = new Path(new BezierLine(new Point(specimen2StrafePose), new Point(specimen2PushPose)));
        specimen2PushPath.setLinearHeadingInterpolation(specimen2StrafePose.getHeading(), specimen2PushPose.getHeading());

        readyForHumanPath1 = new Path(new BezierLine(new Point(specimen2PushPose), new Point(readyForHuman)));
        readyForHumanPath1.setLinearHeadingInterpolation(specimen2PushPose.getHeading(), readyForHuman.getHeading());

        getSpeciPath = new Path(new BezierLine(new Point(readyForHuman), new Point(humanPlayerZonePose)));
        getSpeciPath.setLinearHeadingInterpolation(readyForHuman.getHeading(), humanPlayerZonePose.getHeading());

        getSpeciBackPath = new Path(new BezierLine(new Point(humanPlayerZonePose), new Point(humanPlayerZonebackPose)));
        getSpeciBackPath.setLinearHeadingInterpolation(humanPlayerZonePose.getHeading(), humanPlayerZonebackPose.getHeading());

        specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
        specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());

        returnHomePath = new Path(new BezierLine(new Point(highChamberPose), new Point(returnHomePose)));
        returnHomePath.setLinearHeadingInterpolation(highChamberPose.getHeading(), returnHomePose.getHeading());

        //working to before first push part
        //pushPath = new PathChain(specimen1Path, specimen1StrafePath);

        pushPath = new PathChain(specimen1Path, specimen1StrafePath, specimen1PushPath, specimen2Path, specimen2StrafePath,specimen2PushPath, readyForHumanPath1);
        //pushPath = new PathChain(specimen1Path, specimen1StrafePath, specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath, readyForHumanPath1);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initial state
                if (clawSubsystem.height == 0) {
                    clawSubsystem.moveUpSp();
                    clawSubsystem.extendOriginal();
                    follower.followPath(placeSpecimenPath, true);
                } else if (clawSubsystem.height == 1) {
                    setPathState(1);
                }
                break;
            case 1: // Place specimen
                if (isNearPose(follower.getPose(), placeSpecimenPose, 1.5)) {
                    if (clawSubsystem.height == 1) {
                        clawSubsystem.moveUpSpDownBit();
                        // elevatorUp = false;
                        // opmodeTimer.resetTimer();
                        // setPathState(-1);
                    } else if (clawSubsystem.height == 9) {
                        clawSubsystem.moveUpSpDownBitThenUp();
                        //setPathState(-1);
                    } else if (clawSubsystem.height == 11) {
                        // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                        follower.followPath(pushReadyPath, true);

                        // clawSubsystem.height = 1;
                        setPathState(3);

                    }

                }
                break;

            case 3: // Ready for next phase
                if (isNearPose(follower.getPose(), afterSpecimenPose, 1.5)) {
                    if (clawSubsystem.height != 0) {
                        clawSubsystem.moveDownSp();
                    }
                    // clawSubsystem.moveDownSp();
                    follower.followPath(pushPath, true);
                    opmodeTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5: // Ready for human interaction
                if (isNearPose(follower.getPose(), readyForHuman, 1.5)) {
                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    clawSubsystem.height = 0;
                    setPathState(12);
                }
                break;



            case 12: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 1) {
                    if (clawSubsystem.height == 0) {
                        clawSubsystem.moveUpSp();
                        follower.followPath(getSpeciBackPath, true);
                    }  if (clawSubsystem.height == 1) {

                        x += 3;
                        placeSpecimenPose1 = new Pose(32, 26 - x, 0);
                        specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
                        specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());
                        follower.followPath(specimenToHighChamberPath, true);
                        setPathState(13);

                    }
                }
                break;

            case 13: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose1, 1)) {
                    // Initial state
                    if (clawSubsystem.height == 1) {
                        clawSubsystem.moveUpSpDownBit();
                        // elevatorUp = false;
                        // opmodeTimer.resetTimer();
                        // setPathState(-1);
                    } else if (clawSubsystem.height == 9) {
                        clawSubsystem.moveUpSpDownBitThenUp();
                        //setPathState(-1);
                    } else if (clawSubsystem.height == 11) {
                        // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                        follower.followPath(pushReadyPath, true);

                        // clawSubsystem.height = 1;
                        setPathState(14);

                    }

                }


                break;
            case 14: // Back to play area
                if (isNearPose(follower.getPose(), readyForHuman, 1.5)) {
                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    clawSubsystem.height = 0;
                    setPathState(15);
                }
                break;

            case 15: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                    if (clawSubsystem.height == 0) {
                        clawSubsystem.moveUpSp();
                        follower.followPath(getSpeciBackPath, true);
                    }  if (clawSubsystem.height == 1) {

                        x += 3;
                        placeSpecimenPose1 = new Pose(32, 26 - x, 0);
                        specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
                        specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());
                        follower.followPath(specimenToHighChamberPath, true);
                        setPathState(16);

                    }
                }
                break;

            case 16: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose1, 1)) {
                    // Initial state
                    if (clawSubsystem.height == 0) {
                        clawSubsystem.moveBit();
                        elevatorUp = false;
                        opmodeTimer.resetTimer();
                    } else if (clawSubsystem.height == 1) {
                        setPathState(17);
                    }
                    highChamberCount++;
                    setPathState(-1);
                }


                break;

            case 20: // Return home
                if (isNearPose(follower.getPose(), highChamberPose, 1.5)) {
                    follower.followPath(returnHomePath, true);
                    setPathState(-1); // End
                }
                break;

            case -1: // Autonomous complete
                telemetry.addData("Status", "Autonomous Complete");
                break;
        }
    }

    private boolean isNearPose(Pose current, Pose target, double tolerance) {
        return Math.abs(current.getX() - target.getX()) < tolerance &&
                Math.abs(current.getY() - target.getY()) < tolerance;
    }

    private void setPathState(int state) {
        pathState = state;
    }
}
