package frc.robot.subsystems;
//JsonING becuse thats what it is

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.fasterxml.jackson.core.JsonParser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.MalformedURLException;
import java.net.URL;


public class JsonING extends SubsystemBase {


    public void robotInit(){
        
    }
    
    @Override
    public void periodic(){
        System.out.println("wut?????????????????????????????????????????");
        try
        {
        URL url = new URL("http://wpilibpi.local:8080/home/pi/data.json");
        JSONParser parser = new JSONParser();
        JSONObject json = (JSONObject) parser.parse(new InputStreamReader(url.openStream()));
        System.out.println(json);
        System.out.println("stuff ran, possibly nothing outoput due to compuers being computers?");
        }

        catch(IOException | ParseException e)
        {
            System.out.println("nope");
        }
    }

}