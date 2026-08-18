/* RT-Smart _start_c — bridges from _start (asm) to __libc_start_main.
 *
 * Mirrors musl's crt1.c but separated so we can pair it with our
 * custom _start that handles RT-Smart's register convention. */

extern int main();

extern void _init() __attribute__((weak));
extern void _fini() __attribute__((weak));

int __libc_start_main(
    int (*)(int, char **, char **),
    int, char **,
    void (*)(void), void (*)(void), void (*)(void));

__attribute__((visibility("hidden"), noreturn))
void _start_c(long *p) {
    int argc = (int)p[0];
    char **argv = (char **)(p + 1);
    __libc_start_main(main, argc, argv, _init, _fini, 0);
    __builtin_unreachable();
}
