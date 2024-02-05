package PlanGeneration;

import py4j.GatewayServer;


class PlanGenerationGateway
{
    public int assertAvailable() {return 0;}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new PlanGenerationGateway(), 12345);
        g.start();
        System.out.println("Path Generation server started...");
    }
}