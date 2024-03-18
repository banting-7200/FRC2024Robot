package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// this is only here so git sees this file 
public class PhotonCamera extends SubsystemBase{
        private final NetworkTableInstance instance;
        private NetworkTable cameraTable; 

        public static final String kTableName = "photonvision";
        public static final String cameraName = "photoncam"; 

        private final String path;

        public void test(NetworkTable cameraTable){ 

                this.cameraTable = cameraTable; 
                //contine here without use of subtable 
        }
        

        //using sub table
        public PhotonCamera(NetworkTableInstance instance, NetworkTable cameraTable){

                this.instance = instance; //object of network table 

                var photonvision_root_table = instance.getTable(kTableName); 
                this.cameraTable = photonvision_root_table.getSubTable(cameraName); 
                // Make subtable for distance returns using instance of root table  

                        path = cameraTable.getPath(); //Path to get to sub table 

                //Create a subscriber responsible for getting pose relitive to robot (x, y, z, qw, qx, qy, qz)

                /* 
                var targetPoseEntry =
                cameraTable
                        .getRawTopic("targetPose")
                        .subscribe(
                                "targetPose", new byte[] {}, 
                                PubSubOption.periodic(0.01), 
                                PubSubOption.sendAll(true)); //send every 0.01 seconds and send everything 
                */
            }


        public DoubleArrayTopic getNoteDist(){ //get distance variables 
                /* 
                       (x, y, z, qw, qx, qy, qz)
                        x ~~ forward/backward 
                        y ~~ left/right 
                        z ~~ up/down 
                */

                DoubleArrayTopic targetPose = cameraTable.getDoubleArrayTopic("targetPose");
                return targetPose; 
        }

        public BooleanTopic hasTarget(){ //get if can see note 
 
                BooleanTopic noteTarget = cameraTable.getBooleanTopic("hasTarget");
                return noteTarget; 
        }
}
