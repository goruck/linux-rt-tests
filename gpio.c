/*

gpio.c

compile with "gcc -Wall -o gpio gpio.c -lrt"

Based on Dom and Gert rev 15-feb-13

*/

// Access from ARM Running Linux

#define BCM2708_PERI_BASE        0x3F000000 /* modified for Pi 2 */
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <poll.h>
#include <stdint.h>
#include <unistd.h>

#include <sched.h>
#include <string.h>
#include <time.h>
#include <sys/mman.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0 (BCM Set 0)
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0 (BCM Clear 0)
#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH (BCM Level 0)
#define GPIO_EVENT *(gpio+16) // 0 if no event, (1<<g) if event (BCM Event Detect Status 0)
#define ENB_GPIO_REDGE *(gpio+19) //Rising Edge Detect Enable 0
#define ENB_GPIO_FEDGE *(gpio+22) //Falling Edge Detect Enable
#define ENB_GPIO_HIDET *(gpio+25) //High Detect Enable 0
#define ENB_GPIO_LODET *(gpio+28) //Low Detect Enable 0
#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock (BCM Clock 0)

// GPIO mapping.
#define PI_CLOCK_IN	13
#define PI_DATA_IN	5
#define PI_DATA_OUT	6

// real-time
#define MY_PRIORITY (90)
#define MAX_SAFE_STACK (100*1024) // 100KB
#define NSEC_PER_SEC (1000000000)

// stack_prefault
void stack_prefault(void) {
  unsigned char dummy[MAX_SAFE_STACK];
  memset(dummy, 0, MAX_SAFE_STACK);
  return;
} // stack_prefault

// Set up a memory regions to access GPIO
void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }

   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             	//Any adddress in our space will do
      BLOCK_SIZE,		//Map length
      PROT_READ|PROT_WRITE,	//Enable reading & writting to mapped memory
      MAP_SHARED,	//Shared with other processes
      mem_fd,           	//File to map
      GPIO_BASE         	//Offset to GPIO peripheral
   );

   close(mem_fd); //No need to keep mem_fd open after mmap

   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;

} // setup_io

int main(int argc, char **argv)
{
  #define MAX_DATA (10*1024) //10 KB
  int data[MAX_DATA];
  int *data_ptr, i=0, x=0, y=0;
  int *data_end = data + MAX_DATA - 1;
  FILE *out_file;
  struct sched_param param;
  struct timespec t;
  int interval = 50000; /* 50us */

  /* Declare ourself as a real time task */
  param.sched_priority = MY_PRIORITY;
  if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed");
    exit(-1);
  }

  /* Lock memory */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(-2);
  }

  /* Pre-fault our stack */
  stack_prefault();

  for(data_ptr = data; data_ptr < data_end; ++data_ptr)
    *data_ptr = 0;

  // Set up gpio pointer for direct register access
  setup_io();

  // Set pin direction
  INP_GPIO(PI_DATA_OUT); // must use INP_GPIO before we can use OUT_GPIO
  OUT_GPIO(PI_DATA_OUT);
  INP_GPIO(PI_DATA_IN);
  INP_GPIO(PI_CLOCK_IN);

  // Set PI_DATA_OUT pin low.
  GPIO_CLR = 1<<PI_DATA_OUT;

  // Enable edge detect on PI_CLOCK_IN pin.
  // Clear event by writting 1 to GPIO_EVENT bit of interest.
  //ENB_GPIO_FEDGE = 1<<PI_CLOCK_IN;
  //ENB_GPIO_REDGE = 1<<PI_CLOCK_IN;

  clock_gettime(CLOCK_MONOTONIC, &t);
  /* start after one second */
  t.tv_sec++;
  printf("starting t.tv_sec: %lu\n", t.tv_sec);

  data_ptr = data;
  while(data_ptr < data_end) {
    /* wait until next shot */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    // sample clock
    x = GET_GPIO(PI_CLOCK_IN);
    // wait an interval
    t.tv_nsec += interval;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // sample clock again
    y = GET_GPIO(PI_CLOCK_IN);
    // check for a rising edge
    if (y > x)
    // check for a falling edge
    //if (x > y)
      *data_ptr++ = GET_GPIO(PI_DATA_IN);

    // check for 1 second rollover
    while(t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++; // if interval has crossed over a second boundry, then increment
    }
  }

  printf("ending t.tv_sec: %lu\n", t.tv_sec);

  // dump data array
  if ( (out_file = fopen ("data", "w")) == NULL )
    printf ("*** data could not be opened. \n" );
  else
    for ( i = 0; i < (MAX_DATA - 1); i++ )
      fprintf ( out_file, "Data[%i]: %i\n", i, data[i] );
    fclose (out_file);

  /* Unlock memory */
  if(munlockall() == -1) {
    perror("munlockall failed");
    exit(-2);
  }

  return 0;
} // main
