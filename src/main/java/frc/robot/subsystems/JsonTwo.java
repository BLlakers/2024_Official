package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JsonTwo extends SubsystemBase {
    

    public JsonTwo()
    {
       
        
    }

    public void robotInit(){
        
    }

    

    @Override
    public void periodic() {
        System.out.println("wut?????????????????????????????????????????");
        //try
        //{
        //URL url = new URL("http://wpilibpi.local:8080/home/pi/data.json");
        //JSONParser parser = new JSONParser();
        //JSONObject json = (JSONObject) parser.parse(new InputStreamReader(url.openStream()));
        //System.out.println(json);
        //System.out.println("stuff ran, possibly nothing outoput due to compuers being computers?");
        //}

        //catch(IOException | ParseException e)
        //{
        //    System.out.println("nope");
        //}
        
    }

}
