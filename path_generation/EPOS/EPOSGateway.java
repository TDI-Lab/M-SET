package EPOS;


import py4j.GatewayServer;

//  TODO: Compile EPOS classes into single, runnable jar
//  TODO: Add Gateway server to EPOS


class EPOSGateway
{
    public String start() {return "EPOS Starting...";}

    public String stop() {return "EPOS Stopping...";}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new EPOSGateway(), 8081);
        g.start();
    }
}