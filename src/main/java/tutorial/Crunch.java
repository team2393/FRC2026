package tutorial;

/** Thread demo */
public class Crunch
{
    static long counter = 0;

    static void crunch(String who)
    {
        while (true)
        {
            synchronized (Crunch.class)
            {
                counter = counter + 1;
            }
            System.out.println(who + " is at " + counter);
        }
    }

    public static void main(String[] args)
    {
        for (int i=0; i<100;++i)
        {
            String name = "Thread " + i;
            new Thread(() -> crunch(name)).start();
        }
    }
}
