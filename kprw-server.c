/*

kprw-server.c

compile with "gcc -Wall -o kprw-server kprw-server.c -lrt -lpthread -lwrap"

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
#include <stdint.h>
#include <sched.h>
#include <string.h>
#include <time.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sys/utsname.h>

// server includes and #defines
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#define	PORT_NUM	4746 // port number for server
#define	BUF_LEN		16 // size of string to hole longest message incl '\n'
#define BACKLOG		1 // only allow one client to connect
//#define	_BSD_SOURCE // to get definitions of NI_MAXHOST and NI_MAXSERV from <netdb.h>
#include <netdb.h>
#define ADDRSTRLEN	(NI_MAXHOST + NI_MAXSERV + 10)
#include <tcpd.h> //for hosts_ctl()


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
#define MSG_IO_PRI	(50) // message io thread priority
#define PANEL_IO_PRI	(90) // panel io thread priority
#define MAX_SAFE_STACK  (100*1024) // 100KB
#define NSEC_PER_SEC    (1000000000LU) // 1 second.
#define INTERVAL        (10*1000) // 10 us timeslice.
#define CLK_PER		(1000000L) // 1 ms clock period.
#define HALF_CLK_PER	(500000L) // 0.5 ms half clock period.
#define SAMPLE_OFFSET   (120000L) // 0.12 ms sample offset from clk edge for panel read
#define KSAMPLE_OFFSET	(300000L) // 0.30 ms sample offset from clk edge for keypad read
#define HOLD_DATA	(220000L) // 0.22 ms data hold time from clk edge for keypad write
#define CLK_BLANK	(5000000L) // 5 ms min clock blank.
#define NEW_WORD_VALID	(2500000L) // if a bit arrives > 2.5 ms after last one, declare start of new word.
#define MAX_BITS	(64) // max 64-bit word read from panel
#define MAX_DATA 	(1*1024) // 1 KB data buffer of 64-bit data words - ~66 seconds @ 1 kHz.
#define FIFO_SIZE	(MAX_BITS*MAX_DATA) // FIFO depth
#define MSG_IO_UPDATE   (5000000) // 5 ms message io thread update period

// keypad button bit mappings
// no button:	0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff
#define IDLE	"1111111111111111111111111111111111111111111111111111111111111111"
// *:		0xff 0x94 0x7f 0xff 0xff 0xff 0xff 0xff
#define STAR	"1111111110010100011111111111111111111111111111111111111111111111"
// #:		0xff 0x96 0xff 0xff 0xff 0xff 0xff 0xff
#define POUND	"1111111110010110111111111111111111111111111111111111111111111111"
// 0:		0xff 0x80 0x7f 0xff 0xff 0xff 0xff 0xff
#define ZERO	"1111111110000000011111111111111111111111111111111111111111111111"
// 1:		0xff 0x82 0xff 0xff 0xff 0xff 0xff 0xff
#define ONE	"1111111110000010111111111111111111111111111111111111111111111111"
// 2:		0xff 0x85 0x7f 0xff 0xff 0xff 0xff 0xff
#define TWO	"1111111110000101011111111111111111111111111111111111111111111111"
// 3:		0xff 0x87 0xff 0xff 0xff 0xff 0xff 0xff
#define THREE	"1111111110000111011111111111111111111111111111111111111111111111"
// 4: 		0xff 0x88 0xff 0xff 0xff 0xff 0xff 0xff
#define FOUR	"1111111110001000111111111111111111111111111111111111111111111111"
// 5: 		0xff 0x8b 0x7f 0xff 0xff 0xff 0xff 0xff
#define FIVE	"1111111110001011100011111111111111111111111111111111111111111111"
// 6:		0xff 0x8d 0xff 0xff 0xff 0xff 0xff 0xff
#define SIX	"1111111110001101111111111111111111111111111111111111111111111111"
// 7: 		0xff 0x8e 0x7f 0xff 0xff 0xff 0xff 0xff
#define SEVEN	"1111111110001110111111111111111111111111111111111111111111111111"
// 8: 		0xff 0x91 0x7f 0xff 0xff 0xff 0xff 0xff
#define EIGHT	"1111111110010001011111111111111111111111111111111111111111111111"
// 9: 		0xff 0x93 0xff 0xff 0xff 0xff 0xff 0xff
#define NINE	"1111111110010011111111111111111111111111111111111111111111111111"
// stay:        0xff 0xd7 0xff 0xff 0xff 0xff 0xff 0xff
#define STAY	"1111111111010111111111111111111111111111111111111111111111111111"
// away:	0xff 0xd8 0xff 0xff 0xff 0xff 0xff 0xff
#define AWAY	"1111111111011000111111111111111111111111111111111111111111111111"

// structure to hold a snapshot of the panel status
struct status {
  char ledStatus[50];
  char zone1Status[50];
  char zone2Status[50];
  char zone3Status[50];
  char zone4Status[50];    
};

// global for direct gpio access
volatile unsigned *gpio;

// fifo globals
volatile int m_Read1, m_Write1, m_Read2, m_Write2;
volatile char m_Data1[FIFO_SIZE], m_Data2[FIFO_SIZE];

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
    perror("can't open /dev/mem \n");
    exit(EXIT_FAILURE);
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
    fprintf(stderr, "mmap error %d\n", (int) gpio_map); //errno also set!
    exit(EXIT_FAILURE);
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

static inline unsigned int getBinaryData(char *st, int offset, int length)
{
  unsigned int buf = 0, j;

  for (j = 0; j< length; j++)
  {
    buf <<=1;
    if ( *(st + offset + j) == '1' ) buf |= 1;
  }

  return buf;
}

// fifo1 - stores panel to keypad and keypad to panel data
static inline int pushElement1(char *element, int num) {
  int nextElement, i;
  
  nextElement = (m_Write1 + num) % FIFO_SIZE;

  if (nextElement != m_Read1) // fifo not full
    for (i = 0; i < num; i++)
      m_Data1[m_Write1 + i] = element[i];
  
  // if fifo was full, data will be overwritten

  m_Write1 = nextElement;
  return i; // return number of elements pushed
}

static inline int popElement1(char *element, int num) {
  int nextElement, i;

  if (m_Read1 == m_Write1)
    return 0; // fifo is empty

  nextElement = (m_Read1 + num) % FIFO_SIZE;

  for (i = 0; i < num; i++)
    element[i] = m_Data1[m_Read1 + i];

  m_Read1 = nextElement;
  return i;
}

// fifo2 - stores keypad data to be sent to panel
static inline int pushElement2(char *element, int num) {
  int nextElement, i;
  
  nextElement = (m_Write2 + num) % FIFO_SIZE;

  if (nextElement != m_Read2)
    for (i = 0; i < num; i++)
      m_Data2[m_Write2 + i] = element[i];
  else
    return 0; // fifo is full, data not overwritten

  m_Write2 = nextElement;
  return i;
}

static inline int popElement2(char *element, int num) {
  int nextElement, i;

  if (m_Read2 == m_Write2)
    return 0; // fifo is empty

  nextElement = (m_Read2 + num) % FIFO_SIZE;

  for (i = 0; i < num; i++)
    element[i] = m_Data2[m_Read2 + i];

  m_Read2 = nextElement;
  return i;
}

// Decode bits from panel into commands and messages.
static int decode(char * word, char * msg) {
  int cmd = 0, zones = 0, button = 0;
  char year3[2],year4[2],month[2],day[2],hour[2],minute[2];

  cmd = getBinaryData(word,0,8);
  strcpy(msg, "");
  if (cmd == 0x05) {
    strcpy(msg, "LED Status ");
    if (getBinaryData(word,16,1))
      strcat(msg, "Ready, ");
    else
      strcat(msg, "Not Ready, ");
    if (getBinaryData(word,12,1)) strcat(msg, "Error, ");
    if (getBinaryData(word,13,1)) strcat(msg, "Bypass, ");
    if (getBinaryData(word,14,1)) strcat(msg, "Memory, ");
    if (getBinaryData(word,15,1)) strcat(msg, "Armed, ");
    if (getBinaryData(word,17,1)) strcat(msg, "Program, ");
  }
  else if (cmd == 0xa5) {
    sprintf(year3, "%d", getBinaryData(word,9,4));
    sprintf(year4, "%d", getBinaryData(word,13,4));
    sprintf(month, "%d", getBinaryData(word,19,4));
    sprintf(day, "%d", getBinaryData(word,23,5));
    sprintf(hour, "%d", getBinaryData(word,28,5));
    sprintf(minute, "%d", getBinaryData(word,33,6));
    strcpy(msg, "Date: 20");
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
    strcpy(msg, "Zone1 ");
    zones = getBinaryData(word,41,8);
    if (zones & 1) strcat(msg, "1, ");
    if (zones & 2) strcat(msg, "2, ");
    if (zones & 4) strcat(msg, "3, ");
    if (zones & 8) strcat(msg, "4, ");
    if (zones & 16) strcat(msg, "5, ");
    if (zones & 32) strcat(msg, "6, ");
    if (zones & 64) strcat(msg, "7, ");
    if (zones & 128) strcat(msg, "8 ");
    if (zones == 0) strcat(msg, "Ready ");
  }
  else if (cmd == 0x2d) {
    strcpy(msg, "Zone2 ");
    zones = getBinaryData(word,41,8);
    if (zones & 1) strcat(msg, "9, ");
    if (zones & 2) strcat(msg, "10, ");
    if (zones & 4) strcat(msg, "11, ");
    if (zones & 8) strcat(msg, "12, ");
    if (zones & 16) strcat(msg, "13, ");
    if (zones & 32) strcat(msg, "14, ");
    if (zones & 64) strcat(msg, "15, ");
    if (zones & 128) strcat(msg, "16 ");
    if (zones == 0) strcat(msg, "Ready ");
  }
  else if (cmd == 0x34) {
    strcpy(msg, "Zone3 ");
    zones = getBinaryData(word,41,8);
    if (zones & 1) strcat(msg, "9, ");
    if (zones & 2) strcat(msg, "10, ");
    if (zones & 4) strcat(msg, "11, ");
    if (zones & 8) strcat(msg, "12, ");
    if (zones & 16) strcat(msg, "13, ");
    if (zones & 32) strcat(msg, "14, ");
    if (zones & 64) strcat(msg, "15, ");
    if (zones & 128) strcat(msg, "16 ");
    if (zones == 0) strcat(msg, "Ready ");
  }
  else if (cmd == 0x3e) {
    strcpy(msg, "Zone4 ");
    zones = getBinaryData(word,41,8);
    if (zones & 1) strcat(msg, "9, ");
    if (zones & 2) strcat(msg, "10, ");
    if (zones & 4) strcat(msg, "11, ");
    if (zones & 8) strcat(msg, "12, ");
    if (zones & 16) strcat(msg, "13, ");
    if (zones & 32) strcat(msg, "14, ");
    if (zones & 64) strcat(msg, "15, ");
    if (zones & 128) strcat(msg, "16 ");
    if (zones == 0) strcat(msg, "Ready ");
  }
  else if (cmd == 0x0a)
    strcpy(msg, "Panel Program Mode");
  else if (cmd == 0x63)
    strcpy(msg, "Undefined command from panel");
  else if (cmd == 0x64)
    strcpy(msg, "Undefined command from panel");
  else if (cmd == 0x69)
    strcpy(msg, "Undefined command from panel");
  else if (cmd == 0x5d)
    strcpy(msg, "Undefined command from panel");
  else if (cmd == 0x39)
    strcpy(msg, "Undefined command from panel");
  else if (cmd == 0xb1)
    strcpy(msg, "Undefined command from panel");
  else if (cmd == 0x11)
    strcpy(msg, "Keypad query");
  else if (cmd == 0xff) { // keypad to panel data
    strcpy(msg, "From Keypad ");
    if (getBinaryData(word,8,32) == 0xffffffff)
      strcat(msg, "idle");
    else {
      button = getBinaryData(word,8,20); //bits 11~14 data; 15~16 CRC
      if (button == 0x947ff)
        strcat(msg, "button * pressed");
      else if (button == 0x96fff)
        strcat(msg, "button # pressed");
      else if (button == 0x807ff)
        strcat(msg, "button 0 pressed");
      else if (button == 0x82fff)
        strcat(msg, "button 1 pressed");
      else if (button == 0x857ff)
        strcat(msg, "button 2 pressed");
      else if (button == 0x87fff)
        strcat(msg, "button 3 pressed");
      else if (button == 0x88fff)
        strcat(msg, "button 4 pressed");
      else if (button == 0x8b7ff)
        strcat(msg, "button 5 pressed");
      else if (button == 0x8dfff)
        strcat(msg, "button 6 pressed");
      else if (button == 0x8e7ff)
        strcat(msg, "button 7 pressed");
      else if (button == 0x917ff)
        strcat(msg, "button 8 pressed");
      else if (button == 0x93fff)
        strcat(msg, "button 9 pressed");
      else if (button == 0xd7fff)
        strcat(msg, "stay button pressed");
      else if (button == 0xd8fff)
        strcat(msg, "away button pressed");
      else
        strcat(msg, "unknown keypad msg");
    }
  }
  else
    strcpy(msg, "Unknown command from panel");

  return cmd; // return command associated with the message

} // decode

// panel io thread
static void * panel_io(void *arg) {
  char word[MAX_BITS] = "", wordkw[MAX_BITS], wordkr[MAX_BITS] ="", wordkr_temp = '0';
  int flag = 0, bit_cnt = 0, res;
  struct timespec t, tmark;

  // detach the thread since we don't care about its return status
  res = pthread_detach(pthread_self());
  if (res != 0) {
    perror("panel i/o thread detach failed\n");
    exit(EXIT_FAILURE);
  }

  strncpy(wordkw, IDLE, MAX_BITS);
  clock_gettime(CLOCK_MONOTONIC, &t);
  tmark = t;
  while (1) {
    t.tv_nsec += INTERVAL;
    tnorm(&t);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    if ((GET_GPIO(PI_CLOCK_IN) == PI_CLOCK_HI) && (flag == 0)) { // write/read keypad data
      if (ts_diff(&t, &tmark) > NEW_WORD_VALID) { // check for new word
        res = pushElement1(word, MAX_BITS); // store p->k data
        if (res != MAX_BITS)
          fprintf(stderr, "panel_io: fifo write error\n"); // record error and continue
        res = pushElement1(wordkr, MAX_BITS); // store k->p data
        if (res != MAX_BITS)
          fprintf(stderr, "panel_io: fifo write error\n");
        if (getBinaryData(word,0,8) == 0x05 && getBinaryData(word,32,1) == 0x1) // safe to send keypad data
          res = popElement2(wordkw, MAX_BITS); // get a keypad command to send to panel
          if (res != MAX_BITS) // fifo is empty so output idle instead of repeating previous
            strncpy(wordkw, IDLE, MAX_BITS);
        bit_cnt = 0; // reset bit counter and arrays
        memset(&word, 0, MAX_BITS);
        memset(&wordkr, 0, MAX_BITS);
      }
      tmark = t; // mark new word time
      flag = 1; // set flag to indicate clock was high
      // write keypad data bit to panel once every time clock is high
      if (wordkw[bit_cnt] == '0') // invert
        GPIO_SET = 1<<PI_DATA_OUT; // set GPIO
      else if (wordkw[bit_cnt] == '1') // invert
        GPIO_CLR = 1<<PI_DATA_OUT; // clear GPIO
      else {
        GPIO_CLR = 1<<PI_DATA_OUT; // clear GPIO
        fprintf(stderr, "panel_io: bad element in keypad data array wordk\n");
        //exit(EXIT_FAILURE);
      }
      t.tv_nsec += KSAMPLE_OFFSET;
      tnorm(&t);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // wait KSAMPLE_OFFSET for valid data
      wordkr_temp = (GET_GPIO(PI_DATA_IN) == PI_DATA_HI) ? '0' : '1'; // invert
      t.tv_nsec += HOLD_DATA;
      tnorm(&t);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // wait HOLD_DATA time
      GPIO_CLR = 1<<PI_DATA_OUT; // leave with GPIO cleared
    }
    else if ((GET_GPIO(PI_CLOCK_IN) == PI_CLOCK_LO) && (flag == 1)) { // read panel data
      flag = 0;
      t.tv_nsec += SAMPLE_OFFSET;
      tnorm(&t);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // wait SAMPLE_OFFSET for valid data
      wordkr[bit_cnt] = wordkr_temp;
      word[bit_cnt++] = (GET_GPIO(PI_DATA_IN) == PI_DATA_HI) ? '0' : '1'; // invert
      if (bit_cnt >= MAX_BITS) bit_cnt = (MAX_BITS - 1); // never let bit_cnt exceed MAX_BITS
    }
  }
} // panel_io thread

// message i/o thread
static void * msg_io(void * arg) {
  int cmd, res;
  int data0, data1, data2, data3;
  int data4, data5, data6, data7;
  char msg[50] = "", oldPKMsg[50] = "", oldKPMsg[50] = "";
  char word[MAX_BITS] = "", wordk[MAX_BITS] = "", buf[128] = "";
  long unsigned index = 0;
  struct timespec t;
  struct status * sptr = (struct status *) arg;

  // detach the thread since we don't care about its return status
  res = pthread_detach(pthread_self());
  if (res != 0) {
    perror("message i/o thread detach failed\n");
    exit(EXIT_FAILURE);
  }

  strncpy(wordk, IDLE, MAX_BITS);
  clock_gettime(CLOCK_MONOTONIC, &t);
  while (1) {
    t.tv_nsec += MSG_IO_UPDATE; // thread runs every MSG_IO_UPDATE seconds
    tnorm(&t);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    res = popElement1(word, MAX_BITS);
    if (res != 0 && res != MAX_BITS) // fifo will be empty most of the time
      fprintf(stderr, "msg_io: fifo read error\n"); // record error and continue
    cmd = decode(word, msg); // decode word from panel into a message
    data0 = getBinaryData(word,0,8);  data1 = getBinaryData(word,8,8);
    data2 = getBinaryData(word,16,8); data3 = getBinaryData(word,24,8);
    data4 = getBinaryData(word,32,8); data5 = getBinaryData(word,40,8);
    data6 = getBinaryData(word,48,8); data7 = getBinaryData(word,56,8);
    if ((cmd == 0xff && strcmp(msg, oldKPMsg) != 0) || 
        (cmd != 0xff && strcmp(msg, oldPKMsg) != 0)) { // only output changes
      snprintf(buf, sizeof(buf),
               "index:%lu,%-50s data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
               index++, msg, data0, data1, data2, data3, data4, data5, data6, data7);
      if (cmd == 0x05) strcpy(sptr->ledStatus, msg);
      if (cmd == 0x27) strcpy(sptr->zone1Status, msg);
      if (cmd == 0x2d) strcpy(sptr->zone2Status, msg);
      if (cmd == 0x34) strcpy(sptr->zone3Status, msg);
      if (cmd == 0x3e) strcpy(sptr->zone4Status, msg);
      fputs(buf, stdout);
      fflush(stdout);
    }
    if (cmd == 0xff)
      strcpy(oldKPMsg, msg);
    else
      strcpy(oldPKMsg, msg);
  }
} // msg_io

// server
static int panserv(struct status * pstat) {
  char buffer[BUF_LEN]="", wordk[MAX_BITS] = "";
  char txBuf[250];
  char addrStr[ADDRSTRLEN];
  char host[NI_MAXHOST];
  char service[NI_MAXSERV];
  int listenfd = 0, connfd = 0, res, num;
  socklen_t addrlen;
  struct sockaddr_in server_addr;
  struct sockaddr_in client_addr;

  signal(SIGPIPE, SIG_IGN); // receive EPIPE from a failed write()
  listenfd = socket(AF_INET, SOCK_STREAM, 0);
  if (listenfd == -1) {
    perror("server: could not open socket\n");
    return(1);
  }
  memset(&server_addr, 0, sizeof(server_addr)); 
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(PORT_NUM); 
  res = bind(listenfd, (struct sockaddr *) &server_addr, sizeof(server_addr));
  if (res != 0) {
    perror("server: bind() failed\n");
    return(1);
  }
  res = listen(listenfd, BACKLOG);
  if (res == -1) {
    perror("server: listen() failed\n");
    return(1);
  }
  for (;;) {
    memset(&client_addr, 0, sizeof(client_addr));
    addrlen = sizeof(struct sockaddr_storage);
    connfd = accept(listenfd, (struct sockaddr *) &client_addr, &addrlen);
    if (connfd == -1) {
      perror("server: accept failed\n");
      continue;
    }
    if (getnameinfo ((struct sockaddr *) &client_addr, addrlen,
        host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
      snprintf(addrStr, ADDRSTRLEN, "(%s, %s)", host, service);
    else
      snprintf(addrStr, ADDRSTRLEN, "(?UNKNOWN?)");
    printf("server: connection requested from %s\n", addrStr);
    if (hosts_ctl("kprw-server", STRING_UNKNOWN, host, STRING_UNKNOWN) == 0) {
      fprintf(stderr, "Client %s connection disallowed\n", inet_ntoa(client_addr.sin_addr));
      close(connfd);
      continue;
    }
    memset(&buffer, 0, BUF_LEN);
    res = read(connfd, buffer, (BUF_LEN-1));
    if (res <= 0) {
      perror("server: error reading from socket");
      close(connfd);
      continue;
    }
    snprintf(txBuf, sizeof(txBuf), "%s, %s, %s, %s, %s,",
             pstat->ledStatus, pstat->zone1Status, pstat->zone2Status,
             pstat->zone3Status, pstat->zone4Status);
    res = write(connfd, txBuf, strlen(txBuf));
    if (res <= 0)
      fprintf(stderr, "server: error writing to socket\n");
    res = close(connfd);
    if (res == -1) {
      perror("server: error closing connection");
      continue;
    }
    num = atoi(buffer);
    printf("server: panel received command %s", buffer);
    if (strncmp(buffer, "star", 4) == 0)
       strncpy(wordk, STAR, MAX_BITS);
    else if (strncmp(buffer, "pound", 5) == 0)
       strncpy(wordk, POUND, MAX_BITS);
    else if (strncmp(buffer, "stay", 4) == 0)
       strncpy(wordk, STAY, MAX_BITS);
    else if (strncmp(buffer, "away", 4) == 0)
       strncpy(wordk, AWAY, MAX_BITS);
    else if (strncmp(buffer, "idle", 4) == 0)
       strncpy(wordk, IDLE, MAX_BITS);
    else if (strncmp(buffer, "0", 1) == 0)
       strncpy(wordk, ZERO, MAX_BITS);
    else if (num > 0 || num < 10)
      switch (num) {
        case 1 :
          strncpy(wordk, ONE, MAX_BITS);
          break;
        case 2 :
          strncpy(wordk, TWO, MAX_BITS);
          break;
        case 3 :
          strncpy(wordk, THREE, MAX_BITS);
          break;
        case 4 :
          strncpy(wordk, FOUR, MAX_BITS);
          break;
        case 5 :
          strncpy(wordk, FIVE, MAX_BITS);
          break;
        case 6 :
          strncpy(wordk, SIX, MAX_BITS);
          break;
        case 7 :
          strncpy(wordk, SEVEN, MAX_BITS);
          break;
        case 8 :
          strncpy(wordk, EIGHT, MAX_BITS);
          break;
        case 9 :
          strncpy(wordk, NINE, MAX_BITS);
          break;
        default :
          fprintf(stderr, "server: invalid panel command\n");
          strncpy(wordk, IDLE, MAX_BITS);
      }
    else {
      fprintf(stderr, "server: invalid panel command\n");
      strncpy(wordk, IDLE, MAX_BITS);
    }
    res = pushElement2(wordk, MAX_BITS); // send keypad data to panel
    if (res != MAX_BITS) {
      fprintf(stderr, "server: fifo write error\n");
      return(1);
    }
  }
  return(0);
} // panserv

int main(int argc, char *argv[])
{
  int res, crit1, crit2, flag, i;
  struct sched_param param_main, param_pio;
  struct utsname u;
  struct status pstat;
  pthread_t pio_thread, mio_thread;
  pthread_attr_t my_attr;
  cpu_set_t cpuset_mio, cpuset_pio;
  FILE *fd;

  // Check if running with real-time linux.
  uname(&u);
  crit1 = (int) strcasestr (u.version, "PREEMPT RT");
  if ((fd = fopen("/sys/kernel/realtime","r")) != NULL) {
    crit2 = ((fscanf(fd, "%d", &flag) == 1) && (flag == 1));
    fclose(fd);
  }
  fprintf(stderr, "This is a %s kernel.\n", (crit1 && crit2)  ? "PREEMPT RT" : "vanilla");
  if (!(crit1 && crit2)) {
    fprintf(stderr, "Can't run under a vanilla kernel\n");
    exit(EXIT_FAILURE);
  }

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
    perror("sched_setscheduler failed\n");
    exit(EXIT_FAILURE);
  }

  /* Lock memory to prevent page faults */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
    perror("mlockall failed\n");
    exit(EXIT_FAILURE);
  }

  /* Pre-fault our stack */
  stack_prefault();

  // Set up gpio pointer for direct register access
  setup_io();

  // Set up FIFOs
  m_Read1 = 0;
  m_Read2 = 0;
  m_Write1 = 0;
  m_Write2 = 0;
  for (i = 0; i < MAX_DATA; i++) {
    m_Data1[i] = '0';
    m_Data2[i] = '0';
  }

  // init panel status indicators
  memset(&pstat, 0, sizeof(pstat));

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
  res = pthread_create(&pio_thread, &my_attr, panel_io, NULL);
  if (res != 0) {
    perror("Panel i/o thread creation failed\n");
    exit(EXIT_FAILURE);
  }
  pthread_attr_destroy(&my_attr);

  // create message input / output thread
  pthread_attr_init(&my_attr);
  pthread_attr_setaffinity_np(&my_attr, sizeof(cpuset_mio), &cpuset_mio);
  pthread_attr_setschedpolicy(&my_attr, SCHED_FIFO);
  param_pio.sched_priority = MSG_IO_PRI;
  pthread_attr_setschedparam(&my_attr, &param_pio);
  res = pthread_create(&mio_thread, &my_attr, msg_io, (void *) &pstat);
  if (res != 0) {
    perror("Message i/o thread creation failed\n");
    exit(EXIT_FAILURE);
  }
  pthread_attr_destroy(&my_attr);

  // start server
  panserv(&pstat);

  /* Unlock memory */
  if(munlockall() == -1) {
    perror("munlockall failed\n");
    exit(EXIT_FAILURE);
  }

  return 0;
} // main
