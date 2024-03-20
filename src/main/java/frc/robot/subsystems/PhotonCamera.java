package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonCamera{ 

        private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
        private static NetworkTable table = tableInstance.getTable("photonvision");

        public static NetworkTableEntry has_targets(){ //is target in view?

                //bool
                NetworkTableEntry has_target = table.getEntry("hasTarget");

                return has_target; 
        }

        public static NetworkTableEntry note_positions(){ 

                //(x, y, z, qw, qx, qy, qz)
                NetworkTableEntry note_pos = table.getEntry("targetPose");

                return note_pos; 
        }
}