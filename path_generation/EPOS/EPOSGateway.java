package EPOS;

import java.io.File;
import java.io.IOException;
import java.lang.ProcessBuilder;

import py4j.GatewayServer;


class EPOSGateway
{
    public String start() {return "EPOS Starting...";}

    public String run() throws IOException, InterruptedException {
        ProcessBuilder pb = new ProcessBuilder("/usr/bin/java", "-jar", "IEPOS.jar");
//        System.out.println(System.getProperty("user.dir") + "/path_generation/EPOS");
        pb.directory(new File(System.getProperty("user.dir") + "/path_generation/EPOS"));
        pb.inheritIO();
        Process p = pb.start();
        p.waitFor();
        return String.valueOf(p.exitValue());
    }

    public String stop() {return "EPOS Stopping...";}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new EPOSGateway(), 8081);
        g.start();
    }
}