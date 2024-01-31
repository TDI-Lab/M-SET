package EPOS;

import java.io.File;
import java.io.IOException;
import java.lang.ProcessBuilder;

import py4j.GatewayServer;


class EPOSGateway
{

    public int assertAvailable() {return 0;}

    private void cleanOutput(File outputDir)
    {
        File[] files = outputDir.listFiles();
        if (files == null) { return; }
        for(File f : files)
        {
            if(f.isDirectory())
            {
                this.cleanOutput(f);
            }
            else
            {
                f.delete();
            }
        }
        outputDir.delete();
    }

    private int runEPOS() throws IOException, InterruptedException {
        ProcessBuilder pb = new ProcessBuilder("/usr/bin/java", "-jar", "IEPOS.jar");
        pb.directory(new File(System.getProperty("user.dir") + "/path_generation/EPOS"));
        pb.inheritIO();
        Process p = pb.start();
        p.waitFor();
        return p.exitValue();
    }

    public int run() throws IOException, InterruptedException {
        this.cleanOutput(new File(System.getProperty("user.dir") + "/path_generation/EPOS/output"));
        new File(System.getProperty("user.dir") + "/path_generation/EPOS/output").mkdirs();
        try
        {
            return this.runEPOS();
        }
        catch (IOException | InterruptedException e)
        {
            return -1;
        }

    }

    public static void main(String[] args)
    {
        GatewayServer g = new GatewayServer(new EPOSGateway(), 8081);
        g.start();
    }
}