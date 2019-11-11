package org.firstinspires.ftc.teamcode;

public class RandomTests extends AutoMasterClass{
    private ALLIANCE_COLOR allianceColor = ALLIANCE_COLOR.RED;

    @Override
    public void runOpMode() {
        initialize();


        try {
           strafe(.3,0,"right",50);
        }
        catch (Exception e){
            telemetry.addData("Exception: ", e);
            telemetry.update();
        }

    }
}