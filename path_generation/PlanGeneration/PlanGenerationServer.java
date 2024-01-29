package PlanGeneration;

import py4j.GatewayServer;


class PlanGenerationServer
{
    public String start() {return "Plan Generation Starting...";}

    public String stop() {return "Plan Generation Stopping...";}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new PlanGenerationServer(), 8080);
        g.start();
    }
}