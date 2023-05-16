#include <stdio.h>
#include <string.h>

int main (int ac, char **av)
{
    int addr = 0;
    int c, w, pending, initial;
    FILE *fp;

    if (ac > 1 && strncmp(av[1], "-b", 2) == 0) {
        // start address
        if (sscanf (&av[1][2], "%x", &addr) == 1) {
            av++;
            ac--;
        } else if (ac > 2 && sscanf (av[2], "%x", &addr) == 2) {
            av += 2;
            ac -= 2;
        }
    }
    fprintf (stderr, "addr = %04x\n", addr);
    if (ac == 1) {
        fp = stdin;
    } else if ((fp = fopen (av[1], "rb")) == NULL) {
        fprintf (stderr, "%s: bad file name\n", av[1]);
        return 1;
    }
    // dump it
    initial = 0;
    if (addr & 1) {
        // odd address start
        if ((c = fgetc (fp)) == EOF) {
            return 0;
        }
        if ((addr & 0xff) != 0) {
            // rewind start address - 1
            printf ("=%04X ", addr - 1);
            printf ("%04X ", c);
            addr++;
            initial = 1;
        }
    }
    if (initial == 0) {
        printf ("=%04X ", addr);
        initial = 1;
    }
    while ((c = fgetc (fp)) != EOF) {
        if (initial == 0 && (addr & 0xff) == 0) {
            printf ("=%04X ", addr);
        }
        initial = 0;
        if ((addr & 1) == 0) {
            w = (c<<8)&0xff00;
            pending = 1;
        } else {
            w |= c;
            printf ("%04X ", w);
            pending = 0;

        }
        if ((addr & 0xf) == 0xf) {
            printf ("\n");
        }
        addr++;
    }
    fclose (fp);
    return 0;
}