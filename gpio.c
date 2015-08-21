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

// GPIO pin mapping.
#define PI_CLOCK_IN	13
#define PI_DATA_IN	5
#define PI_DATA_OUT	6

/*
GPIO high and low level mapping.
*/
#define PI_CLOCK_HI (1<<PI_CLOCK_IN)
#define PI_CLOCK_LO (0)
#define PI_DATA_HI  (1<<PI_DATA_IN)
#define PI_DATA_LO  (0)

// invert
#define INV(g,s)	((1<<g) - s)

// real-time
#define MY_PRIORITY     (90)
#define MAX_SAFE_STACK  (100*1024) // 100KB
#define NSEC_PER_SEC    1000000000LU // 1 S.
#define INTERVAL        (10*1000) // 10 uS timeslice.
#define CLK_PER		1000000L // 1 mS clock period.
#define HALF_CLK_PER	500000L // 0.5 mS half clock period.
#define SAMPLE_OFFSET   750000L // 0.75 mS sample offset from rising edge clk
#define CLK_BLANK	5000000L // 5 mS min clock blank.

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

/* using clock_nanosleep of librt */
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
      __const struct timespec *__req,
      struct timespec *__rem);

/* the struct timespec consists of nanoseconds
 * and seconds. if the nanoseconds are getting
 * bigger than 1000000000 (= 1 second) the
 * variable containing seconds has to be
 * incremented and the nanoseconds decremented
 * by 1000000000.
 */
static inline void tnorm(struct timespec *tp)
{
   while (tp->tv_nsec >= NSEC_PER_SEC) {
      tp->tv_nsec -= NSEC_PER_SEC;
      tp->tv_sec++;
   }
}

// Wait for a change in clock level and measure the time it took.
static inline unsigned long waitCLKchange(struct timespec *tp, int currentState)
{
  unsigned long c = 0;

  while (GET_GPIO(PI_CLOCK_IN) == currentState) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, tp, NULL);
    tp->tv_nsec += INTERVAL;
    tnorm(tp);
    c += INTERVAL;
//printf("cs:%i,c:%lu,t.tv_sec:%lu,t.tv_nsec:%lu\n",currentState,c,tp->tv_sec,tp->tv_nsec);
  }

  return c; // time between change in nanoseconds
} // waitCLKchange

static inline long ts_diff(struct timespec *a, struct timespec *b)
{
  long x, y;

  x = (a->tv_sec)*NSEC_PER_SEC + a->tv_nsec;
  y = (b->tv_sec)*NSEC_PER_SEC + b->tv_nsec;

  return (x - y);
}

unsigned int getBinaryData(int st[50], int offset, int length)
{
  unsigned int buf = 0, j;

  for (j = 0; j< length; j++)
  {
    buf <<=1;
    if (st[offset + j] == 1) buf |= 1;
  }

  return buf;
}

int main(int argc, char **argv)
{
  #define MAX_BITS (50) // 50 bit data word
  #define MAX_DATA (2*1024) // 2 KB buffer of 43-bit data words - ~140 seconds
  int data[MAX_DATA][MAX_BITS];
  //int (*data_ptr)[MAX_DATA][MAX_BITS] = &data, (*bit_ptr)[MAX_BITS] = data;
  int i = 0, j = 0, flag = 0, bit_cnt = 0, data_cnt = 0, zones, cmd;
  int data0,data1,data2,data3,data4,data5,data6;
  FILE *out_file;
  struct sched_param param;
  struct timespec t, tmark;
  char msg[100] = "";

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

  for(i = 0; i < MAX_DATA; i++)
    for(j = 0; j < MAX_BITS; j++)
      data[i][j] = 0;

  // Set up gpio pointer for direct register access
  setup_io();

  // Set pin direction
  INP_GPIO(PI_DATA_OUT); // must use INP_GPIO before we can use OUT_GPIO
  OUT_GPIO(PI_DATA_OUT);
  INP_GPIO(PI_DATA_IN);
  INP_GPIO(PI_CLOCK_IN);

  // Set PI_DATA_OUT pin low.
  GPIO_CLR = 1<<PI_DATA_OUT;

  clock_gettime(CLOCK_MONOTONIC, &t);
  tmark = t;
  while (1) {
//printf("pi_data:%i,data[%i][%i]:%i\n",GET_GPIO(PI_DATA_IN),data_cnt,bit_cnt,data[data_cnt][bit_cnt]);
    t.tv_nsec += INTERVAL;
    tnorm(&t);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    if (GET_GPIO(PI_CLOCK_IN) == PI_CLOCK_HI) flag = 1;
    else if ((GET_GPIO(PI_CLOCK_IN) == PI_CLOCK_LO) && (flag == 1)) {
      if (ts_diff(&t, &tmark) > 1100000) { // start new word
        bit_cnt = 0;
        data_cnt++;
      }
      tmark = t;
      flag = 0;
      t.tv_nsec += INTERVAL;
      tnorm(&t);
      t.tv_nsec += INTERVAL;
      tnorm(&t);
      t.tv_nsec += INTERVAL;
      tnorm(&t);
      t.tv_nsec += INTERVAL;
      tnorm(&t);
      /*t.tv_nsec += INTERVAL;
      tnorm(&t);
      t.tv_nsec += INTERVAL;
      tnorm(&t);
      t.tv_nsec += INTERVAL;
      tnorm(&t);
      t.tv_nsec += INTERVAL;
      tnorm(&t);
      t.tv_nsec += INTERVAL;
      tnorm(&t);
      t.tv_nsec += INTERVAL;
      tnorm(&t);*/
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // wait 40 uS for valid data
      data[data_cnt][bit_cnt++] = (GET_GPIO(PI_DATA_IN) == PI_DATA_HI) ? 0 : 1;
    }
    if (data_cnt == MAX_DATA)
      break;
  }

  // decode and write to file
  if ( (out_file = fopen ("data", "w")) == NULL )
    printf ("*** data could not be opened. \n" );
  else
    for ( i = 0; i < MAX_DATA; i++ ) {
      strcpy(msg, "");
      cmd = getBinaryData(data[i],0,8);
      if (cmd == 0x05) {
        strcpy(msg, "LED Status: ");
        if (getBinaryData(data[i],12,1)) strcat(msg, "Error ");
        if (getBinaryData(data[i],13,1)) strcat(msg, "Bypass ");
        if (getBinaryData(data[i],14,1)) strcat(msg, "Memory ");
        if (getBinaryData(data[i],15,1)) strcat(msg, "Armed ");
        if (getBinaryData(data[i],16,1)) strcat(msg, "Ready ");
      }/*
      else if (cmd == 0xa5) {
        char year3 = (char) getBinaryData(data[i],9,4);
        char year4 = (char) getBinaryData(data[i],13,4);
        char month = (char) getBinaryData(data[i],19,4);
        char day = (char) getBinaryData(data[i],23,5);
        char hour = (char) getBinaryData(data[i],28,5);
        char minute = (char) getBinaryData(data[i],33,6);
        strcpy(msg, "Data: 20");
        strcat(msg, year3);
        strcat(msg, year4);
        strcat(msg, "-");
        strcat(msg, month);
        strcat(msg, "-");
        strcat(msg, day);
        strcat(msg, " ");
        strcat(msg, hour);
        strcat(msg, ":");
        strcat(msg, minute);
      }*/
      else if (cmd == 0x27) {
        strcpy(msg, "Zone1: ");
        zones = getBinaryData(data[i],41,8);
        if (zones & 1) strcat(msg, "1 ");
        if (zones & 2) strcat(msg, "2 ");
        if (zones & 4) strcat(msg, "3 ");
        if (zones & 8) strcat(msg, "4 ");
        if (zones & 16) strcat(msg, "5 ");
        if (zones & 32) strcat(msg, "6 ");
        if (zones & 64) strcat(msg, "7 ");
        if (zones & 128) strcat(msg, "8 ");
      }
      else if (cmd == 0x2d) {
        strcpy(msg, "Zone2: ");
        zones = getBinaryData(data[i],41,8);
        if (zones & 1) strcat(msg, "9 ");
        if (zones & 2) strcat(msg, "10 ");
        if (zones & 4) strcat(msg, "11 ");
        if (zones & 8) strcat(msg, "12 ");
        if (zones & 16) strcat(msg, "13 ");
        if (zones & 32) strcat(msg, "14 ");
        if (zones & 64) strcat(msg, "15 ");
        if (zones & 128) strcat(msg, "16 ");
      }
      else if (cmd == 0x34) {
        strcpy(msg, "Zone3: ");
        zones = getBinaryData(data[i],41,8);
        if (zones & 1) strcat(msg, "9 ");
        if (zones & 2) strcat(msg, "10 ");
        if (zones & 4) strcat(msg, "11 ");
        if (zones & 8) strcat(msg, "12 ");
        if (zones & 16) strcat(msg, "13 ");
        if (zones & 32) strcat(msg, "14 ");
        if (zones & 64) strcat(msg, "15 ");
        if (zones & 128) strcat(msg, "16 ");
      }
      else if (cmd == 0x3e) {
        strcpy(msg, "Zone4: ");
        zones = getBinaryData(data[i],41,8);
        if (zones & 1) strcat(msg, "9 ");
        if (zones & 2) strcat(msg, "10 ");
        if (zones & 4) strcat(msg, "11 ");
        if (zones & 8) strcat(msg, "12 ");
        if (zones & 16) strcat(msg, "13 ");
        if (zones & 32) strcat(msg, "14 ");
        if (zones & 64) strcat(msg, "15 ");
        if (zones & 128) strcat(msg, "16 ");
      }
      else
        strcpy(msg, "Unknown command.");
      data0 = getBinaryData(data[i],0,8);  data1 = getBinaryData(data[i],8,8);
      data2 = getBinaryData(data[i],16,8); data3 = getBinaryData(data[i],24,8);
      data4 = getBinaryData(data[i],32,8); data5 = getBinaryData(data[i],40,8);
      data6 = getBinaryData(data[i],48,2);
      fprintf (out_file, "data[%i],cmd:0x%02x,%s\n", i, cmd, msg);
      fprintf (out_file, "***data-all: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",data0,data1,data2,data3,data4,data5,data6);
    }
  fclose (out_file);

  /* Unlock memory */
  if(munlockall() == -1) {
    perror("munlockall failed");
    exit(-2);
  }

  return 0;
} // main
