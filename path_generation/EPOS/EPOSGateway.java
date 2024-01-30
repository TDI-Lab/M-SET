package EPOS;

import java.io.File;
import java.io.IOException;
import java.lang.ProcessBuilder;

import py4j.GatewayServer;


class EPOSGateway
{

    public int assertAvailable() {return 0;}

    public String run() throws IOException, InterruptedException {
        ProcessBuilder pb = new ProcessBuilder("/usr/bin/java", "-jar", "IEPOS.jar");
        pb.directory(new File(System.getProperty("user.dir") + "/path_generation/EPOS"));
        pb.inheritIO();
        Process p = pb.start();
        p.waitFor();
        return String.valueOf(p.exitValue());
    }

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new EPOSGateway(), 8081);
        g.start();
    }
}