import py4j.GatewayServer;


class GFG
{
    public String Message() {return "Hello";}

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new GFG());
        g.start();
        System.out.println("Gateway Server Started");
    }
}