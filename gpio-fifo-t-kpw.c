/*

gpio-fifo-t-kpw.c - test build to write keypad data to panel based off threaded version.

compile with "gcc -Wall -o gpio-fifo-t-kpw gpio-fifo-t-kpw.c -lrt -lpthread"

must run under linux PREEMPT_RT kernel 3.18.9-rt5-v7

Lindo St. Angel 2015. 

*/

// Need to use some non-portable pthread funcations.
#define _GNU_SOURCE 

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

#include <pthread.h>

#include <semaphore.h>

// GPIO Access from ARM Running Linux. Based on Dom and Gert rev 15-feb-13
#define BCM2708_PERI_BASE	0x3F000000 /* modified for Pi 2 */
#define GPIO_BASE		(BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

// size of page to lock in memory for real time operation
#define PAGE_SIZE (4*1024)
// size of memory for direct gpio access
#define BLOCK_SIZE (4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y).
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET *(gpio+7)  // sets bit by writing a 1, writing a 0 has no effect (BCM Set 0)
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
#define PI_CLOCK_IN	(13)
#define PI_DATA_IN	(5)
#define PI_DATA_OUT	(6)

// GPIO high and low level mapping macros.
#define PI_CLOCK_HI (1<<PI_CLOCK_IN)
#define PI_CLOCK_LO (0)
#define PI_DATA_HI  (1<<PI_DATA_IN)
#define PI_DATA_LO  (0)

// GPIO invert macro
#define INV(g,s)	((1<<g) - s)

// real-time
#define MAIN_PRI	(20) // main thread priority
#define MSG_IO_PRI	(20) // message io thread priority
#define PANEL_IO_PRI	(90) // panel io thread priority
#define MAX_SAFE_STACK  (100*1024) // 100KB
#define NSEC_PER_SEC    (1000000000LU) // 1 second.
#define INTERVAL        (10*1000) // 10 us timeslice.
#define CLK_PER		(1000000L) // 1 ms clock period.
#define HALF_CLK_PER	(500000L) // 0.5 ms half clock period.
#define SAMPLE_OFFSET   (120000L) // 0.12 ms sample offset from clk edge
#define HOLD_DATA	(520000L) // 0.52 ms data hold time from clk edge
#define CLK_BLANK	(5000000L) // 5 ms min clock blank.
#define NEW_WORD_VALID	(2500000L) // if a bit arrives > 2.5 ms after last one, declare start of new word.
#define MAX_BITS	(64) // max 64-bit word read from panel
#define MAX_DATA 	(1*1024) // 1 KB data buffer of 64-bit data words - ~66 seconds @ 1 kHz.

// globals for fifo protection
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static int avail = 0;

// global for direct gpio access
volatile unsigned *gpio;

// fifo struct
struct fifo {
  char *buf;
  int head;
  int tail;
  int size;
};

// struct of two fifos to pass to pthreads
struct fifos {
  struct fifo fifo1;
  struct fifo fifo2;
};

// stack_prefault
static void stack_prefault(void) {
  unsigned char dummy[MAX_SAFE_STACK];
  memset(dummy, 0, MAX_SAFE_STACK);
  return;
} // stack_prefault

// Set up a memory regions to access GPIO
static void setup_io(void) {
  int  mem_fd;
  void *gpio_map;

  /* open /dev/mem */
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
    printf("can't open /dev/mem \n");
    exit(-1);
  }

  /* mmap GPIO */
  gpio_map = mmap(
    NULL,             		//Any adddress in our space will do
    BLOCK_SIZE,			//Map length
    PROT_READ|PROT_WRITE,	//Enable reading & writting to mapped memory
    MAP_SHARED,			//Shared with other processes
    mem_fd,           		//File to map
    GPIO_BASE         		//Offset to GPIO peripheral
  );

  close(mem_fd); //No need to keep mem_fd open after mmap

  if (gpio_map == MAP_FAILED) {
    printf("mmap error %d\n", (int)gpio_map);//errno also set!
    exit(-1);
  }

  // Always use volatile pointer!
  gpio = (volatile unsigned *)gpio_map;

  return;

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

static unsigned int getBinaryData(char *st, int offset, int length)
{
  unsigned int buf = 0, j;

  for (j = 0; j< length; j++)
  {
    buf <<=1;
    if ( *(st + offset + j) == '1' ) buf |= 1;
  }

  return buf;
}

// fifo

//This initializes the FIFO structure with the given buffer and size
static void fifo_init(struct fifo *f, char *buf, int size) {
  f->head = 0;
  f->tail = 0;
  f->size = size;
  f->buf = buf;
}

//This reads nbytes bytes from the FIFO
//The number of bytes read is returned
static int fifo_read(struct fifo *f, char *buf, int nbytes) {
  int i;
  char *p;
  p = buf;

  for(i=0; i < nbytes; i++) {
    if(f->tail != f->head) { //see if any data is available
      *p++ = f->buf[f->tail];  //grab a byte from the buffer
      f->tail++;  //increment the tail
      if(f->tail == f->size) //check for wrap-around
        f->tail = 0;
    }
    else
      return i; //number of bytes read
  }

  return nbytes;
}  // fifo_read

//This writes up to nbytes bytes to the FIFO
//If the head runs in to the tail, not all bytes are written
//The number of bytes written is returned
static int fifo_write(struct fifo *f, const char *buf, int nbytes){
  int i;
  const char *p;
  p = buf;

  for(i=0; i < nbytes; i++) {
  //first check to see if there is space in the buffer
    if((f->head + 1 == f->tail) || ((f->head + 1 == f->size) && (f->tail == 0)))
      return i; //no more room
    else {
      f->buf[f->head] = *p++;
      f->head++;  //increment the head
      if(f->head == f->size) //check for wrap-around
        f->head = 0;
    }
  }

  return nbytes;
} // fifo_write

// Decode bits from panel into commands and messages.
static int decode(char * word, char * msg) {
  int cmd = 0, zones = 0;
  char year3[2],year4[2],month[2],day[2],hour[2],minute[2];

  cmd = getBinaryData(word,0,8);
  strcpy(msg, "");
  if (cmd == 0x05) {
    strcpy(msg, "p->k LED Status: ");
    if (getBinaryData(word,12,1)) strcat(msg, "Error ");
    if (getBinaryData(word,13,1)) strcat(msg, "Bypass ");
    if (getBinaryData(word,14,1)) strcat(msg, "Memory ");
    if (getBinaryData(word,15,1)) strcat(msg, "Armed ");
    if (getBinaryData(word,16,1)) strcat(msg, "Ready ");
  }
  else if (cmd == 0xa5) {
    sprintf(year3, "%d", getBinaryData(word,9,4));
    sprintf(year4, "%d", getBinaryData(word,13,4));
    sprintf(month, "%d", getBinaryData(word,19,4));
    sprintf(day, "%d", getBinaryData(word,23,5));
    sprintf(hour, "%d", getBinaryData(word,28,5));
    sprintf(minute, "%d", getBinaryData(word,33,6));
    strcpy(msg, "p->k Date: 20");
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
  }
  else if (cmd == 0x27) {
    strcpy(msg, "p->k Zone1: ");
    zones = getBinaryData(word,41,8);
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
    strcpy(msg, "p->k Zone2: ");
    zones = getBinaryData(word,41,8);
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
    strcpy(msg, "p->k Zone3: ");
    zones = getBinaryData(word,41,8);
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
    strcpy(msg, "p->k Zone4: ");
    zones = getBinaryData(word,41,8);
    if (zones & 1) strcat(msg, "9 ");
    if (zones & 2) strcat(msg, "10 ");
    if (zones & 4) strcat(msg, "11 ");
    if (zones & 8) strcat(msg, "12 ");
    if (zones & 16) strcat(msg, "13 ");
    if (zones & 32) strcat(msg, "14 ");
    if (zones & 64) strcat(msg, "15 ");
    if (zones & 128) strcat(msg, "16 ");
  }
  else if (cmd == 0x0a)
    strcpy(msg, "p->k panel in program mode");
  else if (cmd == 0x63)
    strcpy(msg, "p->k undefined command");
  else if (cmd == 0x64)
    strcpy(msg, "p->k undefined command");
  else if (cmd == 0x69)
    strcpy(msg, "p->k undefined command");
  else if (cmd == 0x5d)
    strcpy(msg, "p->k undefined command");
  else if (cmd == 0x39)
    strcpy(msg, "p->k Keypad query");
  else if (cmd == 0x11)
    strcpy(msg, "p->k Keypad query");
  else if (cmd == 0xff) {
    strcpy(msg, "k->p ");
    if (getBinaryData(word,8,8) == 0xc2)
      strcat(msg, "arm/disarm.");
    else
      strcat(msg, "null or unknown msg.");
  }
  else
    strcpy(msg, "p->k unknown command.");

  return cmd; // return command associated with the message

} // decode

static int hex2bin(char * hex, char * bin) {
  int i, j;

  // convert hex to bin
  for (j = 0; j < 7; j++)
    for (i = 0; i < 16; i++) {
      switch (hex[i]) {
        case '0':
            strcat(bin, "0000"); break;
        case '1':
            strcat(bin, "0001"); break;
        case '2':
            strcat(bin, "0010"); break;
        case '3':
            strcat(bin, "0011"); break;
        case '4':
            strcat(bin, "0100"); break;
        case '5':
            strcat(bin, "0101"); break;
        case '6':
            strcat(bin, "0110"); break;
        case '7':
            strcat(bin, "0111"); break;
        case '8':
            strcat(bin, "1000"); break;
        case '9':
            strcat(bin, "1001"); break;
        case 'a':
            strcat(bin, "1010"); break;
        case 'b':
            strcat(bin, "1011"); break;
        case 'c':
            strcat(bin, "1100"); break;
        case 'd':
            strcat(bin, "1101"); break;
        case 'e':
            strcat(bin, "1110"); break;
        case 'f':
            strcat(bin, "1111"); break;
        default:
            printf("\n Invalid hex digit %c ", hex[i]);
            return 0;
        }
      }

  return 1;
} // hex2 bin

// panel io thread
static void * panel_io(void * f) {
  char word[MAX_BITS] = "", wordk[MAX_BITS] = "";
  int flag = 0, bit_cnt = 0;
  struct timespec t, tmark;
  //struct fifos * fx;
  //struct fifo fifoa, fifob;

  /*fx = (struct fifos *) f;
  fifoa = fx->fifo1; // data from panel to keypad
  fifob = fx->fifo2; // data from keypad to panel */

//printf("size1: %i size2: %i\n", fifoa.size, fifob.size);

  memset(&wordk[0], 0, sizeof(wordk));

  clock_gettime(CLOCK_MONOTONIC, &t);
  tmark = t;
  while (1) {
    t.tv_nsec += INTERVAL;
    tnorm(&t);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    if ((GET_GPIO(PI_CLOCK_IN) == PI_CLOCK_HI) && (flag == 0)) {
      if (ts_diff(&t, &tmark) > NEW_WORD_VALID) { // check for new word
        fifo_write(&(((struct fifos *) f)->fifo1), word, MAX_BITS); // write current panel word to FIFO
        if (pthread_mutex_lock(&mtx) != 0) { // avail global protection mutex
          perror("panel_io: can't lock mutex\n");
          exit(-1);
        }
        avail += 1; // number of words for message i/o thread to process
        if (pthread_mutex_unlock(&mtx) != 0) {
          perror("panel_io: can't unlock mutex\n");
          exit(-1);
        }
        if (pthread_cond_signal(&cond) != 0) { // wake message i/o thread
          perror("panel_io: signal failed\n");
          exit(-1);
        }
        fifo_read(&(((struct fifos *) f)->fifo2), wordk, MAX_BITS); // get a keypad command to send to panel
        bit_cnt = 0; // reset bit counter
        memset(&word[0], 0, sizeof(word)); // clear panel word array
      }
      tmark = t; // mark new word time
      flag = 1; // set flag to indicate clock was high
//printf("bit_cnt %i\n", bit_cnt);
      // write keypad data bit to panel once every time clock is high
      if (wordk[bit_cnt] == '0') // invert
        GPIO_SET = 1<<PI_DATA_OUT; // set GPIO
      else if (wordk[bit_cnt] == '1') // invert
        GPIO_CLR = 1<<PI_DATA_OUT; // clear GPIO
      else {
        GPIO_CLR = 1<<PI_DATA_OUT; // clear GPIO
        perror("panel_io: bad element in keypad data array wordk\n");
        //exit(-1);
      }
      t.tv_nsec += HOLD_DATA;
      tnorm(&t);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // wait HOLD_DATA time
      GPIO_CLR = 1<<PI_DATA_OUT; // leave with GPIO cleared
    }
    else if ((GET_GPIO(PI_CLOCK_IN) == PI_CLOCK_LO) && (flag == 1)) {
      flag = 0;
      t.tv_nsec += SAMPLE_OFFSET;
      tnorm(&t);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // wait SAMPLE_OFFSET for valid data
      word[bit_cnt++] = (GET_GPIO(PI_DATA_IN) == PI_DATA_HI) ? '0' : '1'; // invert
      if (bit_cnt >= MAX_BITS) bit_cnt = (MAX_BITS - 1); // never let bit_cnt exceed MAX_BITS
    }
  }
  pthread_exit("panel io thread finished");
} // panel_io thread

// message i/o thread
static void * msg_io(void * f) {
  int cmd = 0, i = 0;
  int data0 = 0, data1 = 0, data2 = 0, data3 = 0;
  int data4 = 0, data5 = 0, data6 = 0, data7 = 0;
  char msg[50] = "", oldMsg[50] = "";
  char word[MAX_BITS] = "", wordk[MAX_BITS] = "";
  //struct fifos * fx;
  //struct fifo fifoa, fifob;

  // enter into prog mode: 0xff 0x94 0x7f 0xff 0xff 0xff 0xff 0xff
  const char * enter_prog = "1111111110010100011111111111111111111111111111111111111111111111";
  //const char * enter_prog = "1111111111111111111111111111111111111111111111111111111111111111";
  //const char * enter_prog = "0101010101010101010101010101010101010101010101010101010101010101";
  // exit prog mode: 0xff 0x96 0xff 0xff 0xff 0xff 0xff 0xff
  const char * exit_prog  = "1111111110010110111111111111111111111111111111111111111111111111";
  //const char * exit_prog  = "1111111111111111111111111111111111111111111111111111111111111111";
  //const char * exit_prog  = "1010101010101010101010101010101010101010101010101010101010101010";
  // idle:                 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff
  const char * idle = "1111111111111111111111111111111111111111111111111111111111111111";
  // 0 button press: 0xff 0x80 0x7f 0xff 0xff 0xff 0xff 0xff
  const char * zero = "1111111110000000011111111111111111111111111111111111111111111111";
  // 1 button press: 0xff 0x82 0xff 0xff 0xff 0xff 0xff 0xff
  const char * one = "1111111110000010111111111111111111111111111111111111111111111111";
  // 2 button press: 0xff 0x85 0x7f 0xff 0xff 0xff 0xff 0xff
  const char * two = "1111111110000101011111111111111111111111111111111111111111111111";
  // 6 button press: 0xff 0x8d 0xff 0xff 0xff 0xff 0xff 0xff
  const char * six = "1111111110001101111111111111111111111111111111111111111111111111";

  memset(&wordk[0], 0, sizeof(wordk));
  strncpy(wordk, idle, MAX_BITS);

  /*fx = (struct fifos *) f;
  fifoa = fx->fifo1; // data from panel to keypad
  fifob = fx->fifo2; // data from keypad to panel*/

//printf("size1: %i size2: %i\n", fifoa.size, fifob.size);

  while (1) {
      if (pthread_mutex_lock(&mtx) != 0) { // avail global protection mutex
        perror("msg_io: can't lock mutex\n");
        exit(-1);
      }
      while(avail == 0) // wait for signal from panel i/o thread to wake
        if (pthread_cond_wait(&cond, &mtx) != 0) {
          perror("msg_io: wait failed\n");
          exit(-1);
        }
      while(avail > 0) {
        fifo_write(&(((struct fifos *) f)->fifo2), wordk, MAX_BITS); // send keypad data to panel
        fifo_read(&(((struct fifos *) f)->fifo1), word, MAX_BITS); // get panel data to keypads
        cmd = decode(word, msg); // decode word from panel into a message
        data0 = getBinaryData(word,0,8);  data1 = getBinaryData(word,8,8);
        data2 = getBinaryData(word,16,8); data3 = getBinaryData(word,24,8);
        data4 = getBinaryData(word,32,8); data5 = getBinaryData(word,40,8);
        data6 = getBinaryData(word,48,8); data7 = getBinaryData(word,56,8);
        if (strcmp(msg, oldMsg) != 0) {
          printf ("cmd:0x%02x,%-50s", cmd, msg);
          printf (" ***data-all: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",data0,data1,data2,data3,data4,data5,data6,data7);
          //fprintf (out_file, "cmd:0x%02x,%-25s\n", cmd, msg);
          //fprintf (out_file, "***data-all: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",data0,data1,data2,data3,data4,data5,data6);
          //fflush(out_file);
          fflush(stdout); // won't printf results w/o a flush while in the loop
          strcpy(oldMsg, msg);
        }
        avail--;
        // get new keypad data for next time - for now just a test
        if (i == 125)	   strncpy(wordk, one, MAX_BITS);
        else if (i == 150) strncpy(wordk, two, MAX_BITS);
        else if (i == 175) strncpy(wordk, six, MAX_BITS);
        else if (i == 200) strncpy(wordk, one, MAX_BITS);
        else		   strncpy(wordk, idle, MAX_BITS);
        if (i < 300) i++; else i = 0;
        if (pthread_mutex_unlock(&mtx) != 0) {
          perror("msg_io: can't unlock mutex\n");
          exit(-1);
        }
      }
    }

  pthread_exit("message io thread finished");
} // msg_io

int main(int argc, char *argv[])
{
  char panelData[MAX_DATA*MAX_BITS] = "", keypadData[MAX_DATA*MAX_BITS] = "";
  int res = 0;
  struct sched_param param_main, param_pio;
  struct fifo panelFifo, keypadFifo;
  struct fifos f;
  pthread_t pio_thread, mio_thread;
  pthread_attr_t my_attr;
  void *thread_result;
  cpu_set_t cpuset_mio, cpuset_pio;

  // CPU(s) for message i/o thread
  CPU_ZERO(&cpuset_mio);
  CPU_SET(1, &cpuset_mio);
  CPU_SET(2, &cpuset_mio);
  CPU_SET(3, &cpuset_mio);
  // CPU(s) for panel i/o thread
  CPU_ZERO(&cpuset_pio);
  CPU_SET(0, &cpuset_pio);

  /* Declare ourself as a real time task */
  param_main.sched_priority = MAIN_PRI;
  if(sched_setscheduler(0, SCHED_FIFO, &param_main) == -1) {
    perror("sched_setscheduler failed");
    exit(-1);
  }

  /* Lock memory to prevent page faults */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(-2);
  }

  /* Pre-fault our stack */
  stack_prefault();

  // Set up gpio pointer for direct register access
  setup_io();

  // Set up FIFOs
  fifo_init(&panelFifo, panelData, MAX_DATA*MAX_BITS);
  fifo_init(&keypadFifo, keypadData, MAX_DATA*MAX_BITS);
  f.fifo1 = panelFifo;
  f.fifo2 = keypadFifo;

//printf("size1: %i size2: %i\n", f.fifo1.size, f.fifo2.size);

  // Set pin direction
  INP_GPIO(PI_DATA_OUT); // must use INP_GPIO before we can use OUT_GPIO
  OUT_GPIO(PI_DATA_OUT);
  INP_GPIO(PI_DATA_IN);
  INP_GPIO(PI_CLOCK_IN);

  // Set PI_DATA_OUT pin low.
  GPIO_CLR = 1<<PI_DATA_OUT;

  // create panel input / output thread
  pthread_attr_init(&my_attr);
  pthread_attr_setaffinity_np(&my_attr, sizeof(cpuset_pio), &cpuset_pio);
  pthread_attr_setschedpolicy(&my_attr, SCHED_FIFO);
  param_pio.sched_priority = PANEL_IO_PRI;
  pthread_attr_setschedparam(&my_attr, &param_pio);
  res = pthread_create(&pio_thread, &my_attr, panel_io, (void *) &f);
  if (res != 0) {
    perror("Panel i/o thread creation failed");
    exit(EXIT_FAILURE);
  }
  pthread_attr_destroy(&my_attr);

  // create message input / output thread
  pthread_attr_init(&my_attr);
  pthread_attr_setaffinity_np(&my_attr, sizeof(cpuset_mio), &cpuset_mio);
  pthread_attr_setschedpolicy(&my_attr, SCHED_FIFO);
  param_pio.sched_priority = MSG_IO_PRI;
  pthread_attr_setschedparam(&my_attr, &param_pio);
  res = pthread_create(&mio_thread, &my_attr, msg_io, (void *) &f);
  if (res != 0) {
    perror("Message i/o thread creation failed");
    exit(EXIT_FAILURE);
  }
  pthread_attr_destroy(&my_attr);

  //wait for threads to finish
  res = pthread_join(pio_thread, &thread_result);
  if (res != 0) {
    perror("Panel i/o thread join failed");
    exit(EXIT_FAILURE);
  }
  res = pthread_join(mio_thread, &thread_result);
  if (res != 0) {
    perror("Message i/o thread join failed");
    exit(EXIT_FAILURE);
  }

  pthread_mutex_destroy(&mtx);

  /* Unlock memory */
  if(munlockall() == -1) {
    perror("munlockall failed");
    exit(-2);
  }

  return 0;
} // main
