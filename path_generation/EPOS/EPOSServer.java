package EPOS;

import py4j.GatewayServer;


class EPOSServer
{
    public String Message() {return "Hello";}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new EPOSServer());
        g.start();
        System.out.println("Gateway Server Started");
    }
}