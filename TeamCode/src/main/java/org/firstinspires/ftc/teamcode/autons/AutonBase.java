package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@Autonomous(name = "Test_drive", group = "Auton")
// @Autonomous(...) is the other common choice

public class AutonBase extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();




    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
   // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
   // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test Auton", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.loop();

        switch (currentStage){
            case  _unknown:
                currentStage = stage._00_preStart;
                break;
            case _00_preStart:
                currentStage = stage._10_Drive_Out;
                break;
            case _10_Drive_Out:
                robot.driveTrain.CmdDrive(8,0,0.35,0);
                currentStage = stage._100_End;
                break;
            case _20_Turn_To_Backdrop:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(0,0,0.0,0);
                    currentStage = stage._30_Strafe_Left;

                }
                break;
            case _30_Strafe_Left:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(12,-90,0.35,0);
                    currentStage = stage._40_Strafe_Right;
                }
                break;
            case _40_Strafe_Right:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(12,90,0.35,90);
                    currentStage = stage._50_Turn_Away_From_Backdrop;
                }
                break;
            case _50_Turn_Away_From_Backdrop:
                if (robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(0,0,0.0,0);
                    currentStage = stage._60_Drive_To_Wall;
                }

                break;
            case _60_Drive_To_Wall:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(10,-180,0.35,0);
                    currentStage = stage._100_End;
                }

                break;
            case _100_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();


                }







                break;
        }



    }  //  loop

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _10_Drive_Out,
        _20_Turn_To_Backdrop,
        _30_Strafe_Left,
        _40_Strafe_Right,
        _50_Turn_Away_From_Backdrop,
        _60_Drive_To_Wall,
        _70_Turn,
        _80_Drive,
        _100_End


    }
}