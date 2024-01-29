package PlanGeneration;

import py4j.GatewayServer;


class PlanGenerationServer
{
    public String runExperiment() {return "Hello";}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new PlanGenerationServer());
        g.start();
        System.out.println("Plan Generation Server started...");
    }
}