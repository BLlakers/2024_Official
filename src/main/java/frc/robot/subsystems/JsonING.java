package frc.robot.subsystems;
//JsonING becuse thats what it is

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;


public class JsonING{
    public static void main(String[] args) throws IOException, ParseException {
        URL url = new URL("http://example.com/json");
        JSONParser parser = new JSONParser();
        JSONObject json = (JSONObject) parser.parse(new InputStreamReader(url.openStream()));
        System.out.println(json);
    }
}