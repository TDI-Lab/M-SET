package PlanGeneration;

import py4j.GatewayServer;


class PlanGenerationGateway
{
    public String start() {return "Plan Generation Starting...";}

    public String stop() {return "Plan Generation Stopping...";}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new PlanGenerationGateway(), 8080);
        g.start();
    }
}