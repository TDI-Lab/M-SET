package EPOS;

import py4j.GatewayServer;


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