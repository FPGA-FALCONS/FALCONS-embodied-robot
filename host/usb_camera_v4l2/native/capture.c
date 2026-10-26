// Minimal UVC MMAP reader. Build on RK with the existing libc/Linux headers:
// cc -std=c11 -O2 -Wall -Wextra -Werror capture.c -o falcons-uvc-capture
// stdout: fixed 48-byte LE FVC1 header followed by exactly one MJPEG payload.
// Retain driver clock/source flags; a kernel timestamp is not proof of exposure.
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <time.h>
#include <unistd.h>
static volatile sig_atomic_t stopped;
static void halt(int sig){(void)sig;stopped=1;}
static int call(int fd,unsigned long op,void *arg){int rc;do{rc=ioctl(fd,op,arg);}while(rc<0&&errno==EINTR&&!stopped);return rc;}
static void le32(uint8_t *p,uint32_t x){for(int i=0;i<4;i++)p[i]=(uint8_t)(x>>(8*i));}
static void le64(uint8_t *p,uint64_t x){for(int i=0;i<8;i++)p[i]=(uint8_t)(x>>(8*i));}
static int send_all(const void *buffer,size_t length){const uint8_t *p=buffer;while(length&&!stopped){ssize_t n=write(STDOUT_FILENO,p,length);if(n<0&&errno==EINTR)continue;if(n<=0)return -1;p+=n;length-=(size_t)n;}return length?-1:0;}
static unsigned number(const char *s,unsigned max){char *end;errno=0;unsigned long x=strtoul(s,&end,10);if(errno||*end||x<1||x>max)return 0;return (unsigned)x;}
struct mapping {void *data;size_t length;};
int main(int argc,char **argv){
 int fd=-1,result=1,streaming=0,changed=0,old_priority=0;
 struct mapping maps[8]={{0}};unsigned count=0,width,height,fps;int fixed;
 enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
 if(argc!=6 || !(width=number(argv[2],8192)) || !(height=number(argv[3],8192)) || !(fps=number(argv[4],120)) || (strcmp(argv[5],"0")&&strcmp(argv[5],"1")) || (uint64_t)width*height>16777216){fprintf(stderr,"usage: capture DEVICE WIDTH HEIGHT INTEGER_FPS FIXED_0_OR_1\n");return 2;}
 fixed=argv[5][0]=='1';
 if(prctl(PR_SET_PDEATHSIG,SIGTERM)<0||getppid()==1)return 1;
 struct sigaction action={0};action.sa_handler=halt;sigemptyset(&action.sa_mask);sigaction(SIGTERM,&action,0);sigaction(SIGINT,&action,0);signal(SIGPIPE,SIG_IGN);
 fd=open(argv[1],O_RDWR|O_NONBLOCK|O_CLOEXEC);if(fd<0)goto cleanup;
 if(flock(fd,LOCK_EX|LOCK_NB)<0)goto cleanup;
 struct v4l2_capability cap={0};if(call(fd,VIDIOC_QUERYCAP,&cap)<0)goto cleanup;
 uint32_t capabilities=(cap.capabilities&V4L2_CAP_DEVICE_CAPS)?cap.device_caps:cap.capabilities;
 if(!(capabilities&V4L2_CAP_VIDEO_CAPTURE)||!(capabilities&V4L2_CAP_STREAMING)){errno=ENOTSUP;goto cleanup;}
 struct v4l2_format fmt={0};fmt.type=type;fmt.fmt.pix.width=width;fmt.fmt.pix.height=height;fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;fmt.fmt.pix.field=V4L2_FIELD_ANY;
 if(call(fd,VIDIOC_S_FMT,&fmt)<0)goto cleanup;
 if(fmt.fmt.pix.width!=width||fmt.fmt.pix.height!=height||fmt.fmt.pix.pixelformat!=V4L2_PIX_FMT_MJPEG){errno=EPROTO;goto cleanup;}
 struct v4l2_streamparm rate={0};rate.type=type;rate.parm.capture.timeperframe.numerator=1;rate.parm.capture.timeperframe.denominator=fps;
 if(call(fd,VIDIOC_S_PARM,&rate)<0)goto cleanup;
 if(!rate.parm.capture.timeperframe.numerator||rate.parm.capture.timeperframe.denominator!=fps*rate.parm.capture.timeperframe.numerator){errno=EPROTO;goto cleanup;}
 if(fixed){
  struct v4l2_control ctrl={.id=V4L2_CID_EXPOSURE_AUTO_PRIORITY};if(call(fd,VIDIOC_G_CTRL,&ctrl)<0)goto cleanup;
  old_priority=ctrl.value;ctrl.value=0;if(call(fd,VIDIOC_S_CTRL,&ctrl)<0)goto cleanup;changed=1;
  if(call(fd,VIDIOC_G_CTRL,&ctrl)<0||ctrl.value!=0){errno=EPROTO;goto cleanup;}
 }
 struct v4l2_requestbuffers req={0};req.count=4;req.type=type;req.memory=V4L2_MEMORY_MMAP;
 if(call(fd,VIDIOC_REQBUFS,&req)<0)goto cleanup;
 if(req.count<2||req.count>8){errno=ENOBUFS;goto cleanup;}
 count=req.count;
 for(unsigned i=0;i<count;i++){
  struct v4l2_buffer b={0};b.type=type;b.memory=V4L2_MEMORY_MMAP;b.index=i;
  if(call(fd,VIDIOC_QUERYBUF,&b)<0)goto cleanup;
  if(!b.length||b.length>32*1024*1024){errno=EOVERFLOW;goto cleanup;}
  maps[i].length=b.length;maps[i].data=mmap(0,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
  if(maps[i].data==MAP_FAILED){maps[i].data=0;goto cleanup;}
  if(call(fd,VIDIOC_QBUF,&b)<0)goto cleanup;
 }
 if(call(fd,VIDIOC_STREAMON,&type)<0)goto cleanup;
 streaming=1;
 while(!stopped){
  struct pollfd pfd={.fd=fd,.events=POLLIN};int polled=poll(&pfd,1,2000);
  if(polled<0&&errno==EINTR)continue;
  if(polled<=0||pfd.revents&(POLLERR|POLLHUP|POLLNVAL)){errno=ETIMEDOUT;goto cleanup;}
  struct v4l2_buffer b={0};b.type=type;b.memory=V4L2_MEMORY_MMAP;
  if(call(fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)continue;goto cleanup;}
  struct timespec received;if(clock_gettime(CLOCK_MONOTONIC,&received)<0)goto cleanup;
  if(b.index>=count||!b.bytesused||b.bytesused>maps[b.index].length||b.timestamp.tv_sec<0||b.timestamp.tv_usec<0||b.timestamp.tv_usec>=1000000){errno=EPROTO;goto cleanup;}
  if(!(b.flags&V4L2_BUF_FLAG_ERROR)){
   uint8_t header[48]={0};memcpy(header,"FVC1",4);le32(header+4,48);le32(header+8,width);le32(header+12,height);le32(header+16,b.sequence);le32(header+20,b.flags);le32(header+24,b.bytesused);le32(header+28,V4L2_PIX_FMT_MJPEG);
   le64(header+32,(uint64_t)b.timestamp.tv_sec*1000000+(uint64_t)b.timestamp.tv_usec);
   le64(header+40,(uint64_t)received.tv_sec*1000000+(uint64_t)received.tv_nsec/1000);
   if(send_all(header,sizeof(header))<0||send_all(maps[b.index].data,b.bytesused)<0)goto cleanup;
  }
  if(call(fd,VIDIOC_QBUF,&b)<0)goto cleanup;
 }
 result=0;
cleanup:
 if(result&&!stopped)perror("falcons-uvc-capture");
 if(streaming)call(fd,VIDIOC_STREAMOFF,&type);
 for(unsigned i=0;i<count;i++)if(maps[i].data)munmap(maps[i].data,maps[i].length);
 if(stopped)result=0;
 if(changed){struct v4l2_control ctrl={.id=V4L2_CID_EXPOSURE_AUTO_PRIORITY,.value=old_priority};if(call(fd,VIDIOC_S_CTRL,&ctrl)<0){perror("restore exposure priority");result=1;}}
 if(fd>=0)close(fd);
 return result;
}
