#ifdef __GNUC__
#include  <errno.h>
#include  <stdio.h>
#include  <sys/stat.h>
#include  <sys/unistd.h>
#include "main.h"

#undef errno
extern int errno;

/******************************************************************************
 *
 ******************************************************************************/

void stdio_setup()
{
    // Turn off buffers, so I/O occurs immediately
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

/******************************************************************************
 *
 ******************************************************************************/

int _read(int file, char *data, int len)
{
  return -1;
}

/******************************************************************************
 *
 ******************************************************************************/

int _write(int file, char *data, int len)
{
    int bytes_written;

    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }

    for (bytes_written = 0; bytes_written < len; bytes_written++)
    {
        ITM_SendChar(*data);
        data++;
    }

    return bytes_written;
}

/******************************************************************************
 *
 ******************************************************************************/

int _close(int file)
{
    return -1;
}

/******************************************************************************
 *
 ******************************************************************************/

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

/******************************************************************************
 *
 ******************************************************************************/

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

/******************************************************************************
 *
 ******************************************************************************/

int _isatty(int file)
{
    if ((file == STDOUT_FILENO) ||
        (file == STDIN_FILENO) ||
        (file == STDERR_FILENO))
    {
        return 1;
    }

    errno = EBADF;
    return 0;
}
#else
void stdio_setup() { }
#endif
