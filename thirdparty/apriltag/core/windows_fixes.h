#ifndef _WINDOWS_H_
#define _WINDOWS_H_

#ifdef __WIN32__

#include <errno.h>
#include <stdlib.h>

typedef long long suseconds_t;

static long int random(void)
{
  return rand();
}

static void srandom(unsigned int seed)
{
  srand(seed);
}

static void *my_aligned_alloc(size_t align, size_t size)
{
  int align_not_power_of_2 = align & (align - 1);
  int size_not_multiple_of_align = size & (align - 1);
  if (align_not_power_of_2 || size_not_multiple_of_align)
  {
    errno = EINVAL;
    return NULL;
  }

#if defined(__MINGW32__)
  return __mingw_aligned_malloc(size, align);
#else
  assert(align <= alignof (max_align_t));
  return malloc(((void) align, size));
#endif
}


static int my_memalign(void** pptr, size_t alignment, size_t size)
{
  *pptr = my_aligned_alloc(alignment, size);
  int fail = (!*pptr);
  return fail;
}

#endif  // __WIN32__

#endif  // _WINDOWS_H_