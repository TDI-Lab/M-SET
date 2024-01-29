package EPOS;

import py4j.GatewayServer;

//  TODO: Compile EPOS classes into single, runnable jar
//  TODO: Add Gateway server to EPOS


class EPOSServer
{
    public String runExperiment() {return "Hello";}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new EPOSServer());
        g.start();
        System.out.println("EPOS Server started...");
    }
}