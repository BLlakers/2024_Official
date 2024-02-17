package frc.robot.subsystems;
//JsonING becuse thats what it is

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.JSONObject; 
import org.json.simple.parser.*; 

public class JsonING extends SubsystemBase {
    //only periodic since we only need things to repeat    
    @Override
    public void periodic(){
        //create object here
        Object obj;

        try{
        //update iot here if it dosent fail
        obj = new JSONParser().parse(new FileReader("test.json"));
        }
        catch(IOException | ParseException e){
            System.out.println("it no working");
            return;
        }
        //getting the whole json thing
        JSONObject fromcamera = (JSONObject) obj;

        //getting the values of things
        String x = (String) fromcamera.get("x");
        String y = (String) fromcamera.get("y");
        String obj_name = (String) fromcamera.get("object_name");
        
        //maybe print it out
        System.out.println(x);
        System.out.println(y);
        System.out.println(obj_name);
    }
}